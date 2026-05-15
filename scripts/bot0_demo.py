#!/usr/bin/env python3
import os
import sys
import rospy
import warnings

warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
except ImportError:
    print("stable-baselines3 not installed. Run: pip3 install stable-baselines3")
    sys.exit(1)

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

try:
    from hazard_env import HazardWorldEnv
except ImportError:
    print("Cannot find hazard_env.py in the same directory.")
    sys.exit(1)


def main():
    rospy.init_node("hazard_ppo_demo", anonymous=True)

    model_path   = os.path.join(script_dir, "hazard_rl_model")
    vecnorm_path = model_path + "_vecnormalize.pkl"

    for path in (model_path + ".zip", vecnorm_path):
        if not os.path.exists(path):
            print(f"ERROR: required file not found: {path}")
            sys.exit(1)

    print("\n" + "=" * 60)
    print("  DEMO: PPO policy inference — fixed gas positions")
    print(f"  Model:   {model_path}.zip")
    print(f"  VecNorm: {vecnorm_path}")
    print("=" * 60 + "\n")

    raw_env = DummyVecEnv([lambda: Monitor(HazardWorldEnv(randomise_gas=False, oscillate_fire=True))])

    env = VecNormalize.load(vecnorm_path, raw_env)
    env.training    = False
    env.norm_reward = False

    model = PPO.load(model_path, env=env, device="cpu")

    obs       = env.reset()
    done      = False
    step      = 0
    total_rew = 0.0
    infos     = [{}]

    try:
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, rewards, dones, infos = env.step(action)
            done       = bool(dones[0])
            total_rew += float(rewards[0])
            step      += 1
    except (KeyboardInterrupt, SystemExit):
        print("\n[demo] Episode stopped early")
    finally:
        env.close()
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Demo complete")

    if step > 0:
        print("\n" + "=" * 60)
        print(f"  Steps: {step}  normalised_reward: {total_rew:.3f}")
        if infos and "episode" in infos[0]:
            ep = infos[0]["episode"]
            print(f"  Monitor: raw_reward={ep['r']:.2f}  length={ep['l']}")
        print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
