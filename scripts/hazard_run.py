#!/usr/bin/env python3
"""Run the trained PPO model in Gazebo for evaluation — no training updates.

Loads the backed-up SOLVED model. Runs for a fixed number of episodes,
logs per-episode statistics, and prints aggregate results at the end.

Useful for testing the policy against different gas/fire spawn configurations
without altering the trained weights.
"""
import os
import sys
import time
import signal
import argparse
import numpy as np
import rospy

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
except ImportError:
    print("stable-baselines3 not installed. Run: pip3 install stable-baselines3")
    sys.exit(1)

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from hazard_env import HazardWorldEnv


# ---------------------------------------------------------------- #
#  Model paths — defaults to backed-up SOLVED policy                #
# ---------------------------------------------------------------- #
DEFAULT_MODEL_DIR  = os.path.join(script_dir, "solved_policy_backup")
DEFAULT_MODEL_NAME = "hazard_rl_model_SOLVED"


def parse_args():
    p = argparse.ArgumentParser(description="Evaluate trained PPO policy.")
    p.add_argument("--model-dir", default=DEFAULT_MODEL_DIR,
                   help="Directory containing model .zip and VecNormalize .pkl")
    p.add_argument("--model-name", default=DEFAULT_MODEL_NAME,
                   help="Model file stem (without .zip)")
    p.add_argument("--episodes", type=int, default=20,
                   help="Number of episodes to run (default: 20)")
    p.add_argument("--stochastic", action="store_true",
                   help="Use stochastic action sampling instead of deterministic")
    return p.parse_args()


def main():
    args = parse_args()

    model_path   = os.path.join(args.model_dir, args.model_name)
    vecnorm_path = os.path.join(args.model_dir, args.model_name + "_vecnormalize.pkl")

    if not os.path.exists(model_path + ".zip"):
        print(f"❌ No model found at {model_path}.zip")
        print(f"   Looked in: {args.model_dir}")
        print(f"   Files there: {os.listdir(args.model_dir) if os.path.isdir(args.model_dir) else '(directory does not exist)'}")
        sys.exit(1)

    if not os.path.exists(vecnorm_path):
        print(f"⚠️  No VecNormalize stats at {vecnorm_path}")
        print(f"   The policy was trained with normalised observations; running without normalisation will give wrong behaviour.")
        print(f"   Aborting.")
        sys.exit(1)

    rospy.init_node("hazard_ppo_runner", anonymous=True)

    # ---------------------------------------------------------------- #
    #  Environment setup                                                 #
    # ---------------------------------------------------------------- #
    raw_env = DummyVecEnv([lambda: HazardWorldEnv()])

    print(f"Loading VecNormalize stats: {vecnorm_path}")
    env = VecNormalize.load(vecnorm_path, raw_env)
    env.training    = False   # freeze running stats during inference
    env.norm_reward = False   # don't normalise reward — we want raw values

    print(f"Loading model: {model_path}.zip")
    model = PPO.load(model_path, env=env, device="cpu")

    deterministic = not args.stochastic

    # ---------------------------------------------------------------- #
    #  Run episodes                                                      #
    # ---------------------------------------------------------------- #
    print()
    print("=" * 70)
    print(f"  EVALUATION RUN — {args.episodes} episodes")
    print(f"  Action mode: {'deterministic' if deterministic else 'stochastic'}")
    print(f"  Gas spawn config: read from spawn_config.env")
    print("=" * 70)
    print()

    # Per-episode results
    results = []  # list of dicts

    # Graceful shutdown on Ctrl+C
    interrupted = {"flag": False}
    def _sigint(sig, frame):
        print("\n[runner] Ctrl+C received — finishing current episode then stopping.")
        interrupted["flag"] = True
    signal.signal(signal.SIGINT, _sigint)

    t_start = time.time()

    for ep in range(1, args.episodes + 1):
        if interrupted["flag"]:
            break

        obs = env.reset()
        ep_reward = 0.0
        step      = 0
        done      = False
        t_ep      = time.time()

        # Track gas-finds and termination from underlying env
        base_env = env.venv.envs[0].unwrapped  # peel Monitor → HazardWorldEnv

        print(f"--- Episode {ep:>2}/{args.episodes} start ---")

        while not done and not rospy.is_shutdown() and not interrupted["flag"]:
            action, _ = model.predict(obs, deterministic=deterministic)
            obs, reward, done, info = env.step(action)
            ep_reward += float(reward[0])
            step += 1

        ep_wall_time = time.time() - t_ep
        gas_found    = sum(base_env.gas_visited)
        all_found    = base_env.all_gas_found

        # Mission complete = all gas found AND ep_reward includes the +40 terminal
        # We can detect it by checking gas + distance to home at termination
        if base_env.pose is not None:
            import math
            dist_home = math.hypot(
                base_env.pose.position.x - base_env.home[0],
                base_env.pose.position.y - base_env.home[1]
            )
        else:
            dist_home = -1.0

        mission_complete = all_found and dist_home < 2.0

        results.append({
            "episode":          ep,
            "steps":            step,
            "reward":           ep_reward,
            "gas_found":        gas_found,
            "mission_complete": mission_complete,
            "dist_home":        dist_home,
            "wall_time_s":      ep_wall_time,
        })

        status_emoji = "✓" if mission_complete else ("·" if gas_found > 0 else "✗")
        print(f"--- Episode {ep:>2} end {status_emoji}  steps={step:>4}  "
              f"gas={gas_found}/3  reward={ep_reward:+7.1f}  "
              f"dist_home={dist_home:5.2f}m  time={ep_wall_time:5.1f}s ---\n")

    total_time = time.time() - t_start

    # ---------------------------------------------------------------- #
    #  Aggregate statistics                                              #
    # ---------------------------------------------------------------- #
    if not results:
        print("No episodes completed.")
        env.close()
        return

    rewards   = np.array([r["reward"]           for r in results])
    steps     = np.array([r["steps"]            for r in results])
    gas       = np.array([r["gas_found"]        for r in results])
    missions  = np.array([r["mission_complete"] for r in results])

    print()
    print("=" * 70)
    print(f"  EVALUATION SUMMARY — {len(results)} episodes in {total_time:.1f}s")
    print("=" * 70)
    print(f"  Mission completion rate: {missions.sum()}/{len(results)}  ({100*missions.mean():.1f}%)")
    print(f"  Gas finds per episode:   mean {gas.mean():.2f}/3   median {int(np.median(gas))}/3")
    print(f"    All 3 gas found:       {(gas == 3).sum()}/{len(results)}")
    print(f"    At least 1 gas found:  {(gas >= 1).sum()}/{len(results)}")
    print(f"  Reward:                  mean {rewards.mean():+.1f} ± {rewards.std():.1f}")
    print(f"                           min  {rewards.min():+.1f}   max {rewards.max():+.1f}")
    print(f"  Episode length:          mean {steps.mean():.0f}   min {steps.min()}   max {steps.max()}")
    if missions.sum() > 0:
        mission_steps = steps[missions]
        print(f"  Steps to mission complete (successful eps only):")
        print(f"                           mean {mission_steps.mean():.0f}   min {mission_steps.min()}   max {mission_steps.max()}")
    print("=" * 70)
    print()

    env.close()


if __name__ == "__main__":
    main()