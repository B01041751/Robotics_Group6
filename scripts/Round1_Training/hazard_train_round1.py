#!/usr/bin/env python3
import os
import sys
import time
import rospy
import warnings

warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
except ImportError:
    print("❌ stable-baselines3 not installed. Run: pip3 install stable-baselines3")
    sys.exit(1)

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

try:
    from hazard_env import HazardWorldEnv
except ImportError:
    print("❌ Cannot find hazard_env.py in the same directory.")
    sys.exit(1)


TOTAL_STEPS     = 600_000
CHECKPOINT_FREQ =   5_000


class ProgressCallback(BaseCallback):
    """Prints a one-line progress bar to stdout every N steps."""

    PRINT_EVERY = 500   # console update frequency (steps)

    def __init__(self, total_steps: int, checkpoint_freq: int, verbose=0):
        super().__init__(verbose)
        self.total_steps     = total_steps
        self.checkpoint_freq = checkpoint_freq
        self._t0             = None
        self._ep_rewards     = []
        self._ep_count       = 0

    def _on_training_start(self):
        self._t0 = time.time()
        print("\n" + "=" * 70)
        print(f"  Training started — {self.total_steps:,} total steps")
        print(f"  Checkpoint every {self.checkpoint_freq:,} steps")
        print("=" * 70)

    def _on_step(self) -> bool:
        # Collect episode rewards from the Monitor wrapper
        infos = self.locals.get("infos", [])
        for info in infos:
            if "episode" in info:
                self._ep_rewards.append(info["episode"]["r"])
                self._ep_count += 1

        if self.num_timesteps % self.PRINT_EVERY != 0:
            return True

        step      = self.num_timesteps
        elapsed   = time.time() - self._t0
        sps       = step / elapsed if elapsed > 0 else 0
        remaining = (self.total_steps - step) / sps if sps > 0 else 0
        pct       = step / self.total_steps * 100
        til_save  = self.checkpoint_freq - (step % self.checkpoint_freq)

        bar_len   = 30
        filled    = int(bar_len * step / self.total_steps)
        bar       = "#" * filled + "-" * (bar_len - filled)

        avg_rew   = (sum(self._ep_rewards[-20:]) / len(self._ep_rewards[-20:])
                     if self._ep_rewards else float("nan"))

        eta_h, eta_m = divmod(int(remaining), 3600)
        eta_m //= 60

        print(
            f"[{bar}] {pct:5.1f}%  "
            f"step {step:>7,}/{self.total_steps:,}  "
            f"{sps:5.1f} sps  "
            f"ETA {eta_h}h{eta_m:02d}m  "
            f"next save in {til_save:,}  "
            f"ep {self._ep_count}  "
            f"avg_rew(20) {avg_rew:.1f}"
        )
        return True


class HourlySaveCallback(BaseCallback):
    """Saves model + VecNormalize stats once per hour of wall time."""

    def __init__(self, model_path: str, vecnorm_path: str, interval_s: int = 3600, verbose=0):
        super().__init__(verbose)
        self.model_path   = model_path
        self.vecnorm_path = vecnorm_path
        self.interval_s   = interval_s
        self._last_save   = None

    def _on_training_start(self):
        self._last_save = time.time()

    def _on_step(self) -> bool:
        now = time.time()
        if now - self._last_save >= self.interval_s:
            self.model.save(self.model_path + "_latest")
            if hasattr(self.training_env, 'save'):
                self.training_env.save(self.vecnorm_path + ".latest")
            self._last_save = now
            print(f"\n[HourlySave] Saved latest model at step {self.num_timesteps:,}\n")
        return True


def make_env(log_dir):
    return Monitor(HazardWorldEnv(), filename=os.path.join(log_dir, "monitor.csv"))


def main():
    rospy.init_node("hazard_ppo_trainer", anonymous=True)

    log_dir        = os.path.join(script_dir, "logs")
    checkpoint_dir = os.path.join(script_dir, "checkpoints")
    os.makedirs(log_dir,        exist_ok=True)
    os.makedirs(checkpoint_dir, exist_ok=True)

    model_path      = os.path.join(script_dir, "hazard_rl_model")
    vecnorm_path    = model_path + "_vecnormalize.pkl"
    tensorboard_dir = os.path.join(script_dir, "ppo_hazard_tensorboard")

    use_tb = True
    try:
        import tensorboard
    except ImportError:
        print("⚠️  tensorboard not installed — logging to stdout only.")
        use_tb = False

    # ------------------------------------------------------------------ #
    #  Build vectorised + normalised environment                           #
    # ------------------------------------------------------------------ #
    raw_env = DummyVecEnv([lambda: make_env(log_dir)])

    if os.path.exists(vecnorm_path):
        print(f"--- LOADING VecNormalize stats: {vecnorm_path} ---")
        try:
            env = VecNormalize.load(vecnorm_path, raw_env)
            expected = raw_env.observation_space.shape
            if env.obs_rms.mean.shape != expected:
                raise ValueError(
                    f"obs shape mismatch: pkl={env.obs_rms.mean.shape} env={expected}"
                )
            env.training    = True
            env.norm_reward = True
        except Exception as e:
            print(f"⚠️  VecNormalize load failed ({e}) — discarding stale stats.")
            os.rename(vecnorm_path, vecnorm_path + ".incompatible")
            env = VecNormalize(
                raw_env, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.98
            )
    else:
        print("--- CREATING new VecNormalize ---")
        env = VecNormalize(
            raw_env,
            norm_obs=True,
            norm_reward=True,
            clip_obs=10.0,        # clip normalised obs to [-10, 10]
            gamma=0.98,           # must match PPO gamma for correct return normalisation
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    checkpoint_callback = CheckpointCallback(
        save_freq=CHECKPOINT_FREQ,
        save_path=checkpoint_dir,
        name_prefix="hazard_model",
    )
    progress_callback = ProgressCallback(
        total_steps=TOTAL_STEPS,
        checkpoint_freq=CHECKPOINT_FREQ,
    )
    hourly_callback = HourlySaveCallback(
        model_path=model_path,
        vecnorm_path=vecnorm_path,
        interval_s=3600,
    )

    # ------------------------------------------------------------------ #
    #  Load or create PPO model                                            #
    # ------------------------------------------------------------------ #
    full_zip_path = model_path + ".zip"
    model = None

    if os.path.exists(full_zip_path):
        print(f"--- LOADING EXISTING MODEL: {full_zip_path} ---")
        try:
            model = PPO.load(
                model_path,
                env=env,
                device="cpu",
                tensorboard_log=tensorboard_dir if use_tb else None,
            )
        except Exception as e:
            print(f"⚠️  Could not load model ({e}). Starting fresh.")
            os.rename(full_zip_path, full_zip_path + ".incompatible")
            model = None

    if model is None:
        print("--- CREATING NEW MODEL ---")
        model = PPO(
            policy="MlpPolicy",
            env=env,
            verbose=1,
            learning_rate=3e-4,
            n_steps=2048,        # smaller buffer → more frequent updates → faster feedback
            batch_size=64,
            n_epochs=10,
            gamma=0.98,
            ent_coef=0.05,       # higher entropy → more exploration of large arena
            clip_range=0.2,
            tensorboard_log=tensorboard_dir if use_tb else None,
            device="cpu",
        )

    print(f"--- STARTING TRAINING on {model.device} ---")

    try:
        model.learn(
            total_timesteps=TOTAL_STEPS,
            callback=[checkpoint_callback, progress_callback, hourly_callback],
            log_interval=1,
            reset_num_timesteps=False,
        )
        model.save(model_path)
        env.save(vecnorm_path)
        print("--- SUCCESS: model and VecNormalize stats saved ---")

    except KeyboardInterrupt:
        print("\n--- STOPPED BY USER — saving checkpoint ---")
        model.save(model_path + "_interrupted")
        env.save(vecnorm_path + ".interrupted")
        print(f"--- Saved to {model_path}_interrupted ---")

    except Exception as e:
        print(f"--- CRITICAL ERROR: {e} ---")

    finally:
        env.close()
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Training finished")


if __name__ == "__main__":
    main()
