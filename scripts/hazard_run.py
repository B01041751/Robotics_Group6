#!/usr/bin/env python3
"""Run the trained PPO model in Gazebo — no training, observation only."""
import os
import sys
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

model_path   = os.path.join(script_dir, "hazard_rl_model")
vecnorm_path = model_path + "_vecnormalize.pkl"


def main():
    rospy.init_node("hazard_ppo_runner", anonymous=True)

    if not os.path.exists(model_path + ".zip"):
        print(f"No model found at {model_path}.zip — train first.")
        sys.exit(1)

    raw_env = DummyVecEnv([lambda: HazardWorldEnv()])

    if os.path.exists(vecnorm_path):
        print(f"Loading VecNormalize stats: {vecnorm_path}")
        env = VecNormalize.load(vecnorm_path, raw_env)
        env.training   = False   # freeze running stats during inference
        env.norm_reward = False  # don't normalise reward — we want raw values
    else:
        print("No VecNormalize stats found — running without normalisation.")
        env = raw_env

    print(f"Loading model: {model_path}.zip")
    model = PPO.load(model_path, env=env, device="cpu")

    print("\n=== RUNNING TRAINED MODEL (Ctrl-C to stop) ===\n")

    ep = 0
    while not rospy.is_shutdown():
        obs = env.reset()
        ep += 1
        step = 0
        ep_reward = 0.0
        done = False

        print(f"--- Episode {ep} start ---")

        while not done and not rospy.is_shutdown():
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            ep_reward += float(reward[0])
            step += 1

        print(f"--- Episode {ep} end — steps={step}  total_reward={ep_reward:.1f} ---\n")

    env.close()


if __name__ == "__main__":
    main()
