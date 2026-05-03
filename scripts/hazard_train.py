#!/usr/bin/env python3
import os
import sys
import rospy
import warnings
import numpy as np

# Suppress Gym/Gymnasium and NumPy 2.0 warnings to keep the terminal clean
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.callbacks import CheckpointCallback
except ImportError:
    print("❌ ERROR: stable-baselines3 is not installed. Run: pip3 install stable-baselines3")
    sys.exit(1)

# Set up local imports
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

try:
    from hazard_env import HazardWorldEnv
except ImportError:
    print("❌ ERROR: Cannot find hazard_env.py in the same directory.")
    sys.exit(1)

def main():
    # 1. Initialize ROS Node
    rospy.init_node("hazard_ppo_trainer", anonymous=True)
    
    # 2. Setup Logging Directories
    log_dir = os.path.join(script_dir, "logs")
    checkpoint_dir = os.path.join(script_dir, "checkpoints")
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    # Wrap environment with Monitor for SB3 logging
    env = Monitor(HazardWorldEnv(), filename=os.path.join(log_dir, "monitor.csv"))
    
    model_path = os.path.join(script_dir, "hazard_rl_model")
    tensorboard_path = os.path.join(script_dir, "ppo_hazard_tensorboard")

    # 3. Check for Tensorboard installation to prevent crashes
    use_tb = True
    try:
        import tensorboard
    except ImportError:
        print("⚠️ WARNING: Tensorboard not installed. Logging to stdout only.")
        print("   To fix this, run: pip3 install tensorboard")
        use_tb = False

    # 4. Define Checkpoint Callback
    checkpoint_callback = CheckpointCallback(
        save_freq=50000, 
        save_path=checkpoint_dir,
        name_prefix="hazard_model"
    )

    # 5. Load or Create Model
    # Note: .zip is added automatically by SB3, we check for existence
    full_zip_path = model_path + ".zip"
    
    if os.path.exists(full_zip_path):
        print(f"--- LOADING EXISTING MODEL: {full_zip_path} ---")
        model = PPO.load(
            model_path, 
            env=env, 
            device="cpu", 
            tensorboard_log=tensorboard_path if use_tb else None
        )
    else:
        print("--- CREATING NEW HIGH-SPEED MODEL ---")
        model = PPO(
            policy="MlpPolicy",
            env=env,
            verbose=1,
            learning_rate=3e-4, 
            n_steps=4096,       
            batch_size=128,     
            n_epochs=10,        
            gamma=0.98,         
            ent_coef=0.01,      
            clip_range=0.2,
            tensorboard_log=tensorboard_path if use_tb else None,
            device="cpu"
        )

    print(f"--- STARTING TRAINING ON {model.device} ---")
    
    try:
        # 500,000 steps for robust learning of dynamic obstacles
        model.learn(
            total_timesteps=500000, 
            callback=checkpoint_callback,
            log_interval=1, 
            reset_num_timesteps=False
        )
        model.save(model_path)
        print("--- SUCCESS: FINAL MODEL SAVED ---")
        
    except KeyboardInterrupt:
        print("\n--- STOPPED BY USER: SAVING EMERGENCY BACKUP ---")
        model.save(model_path + "_interrupted")
        
    except Exception as e:
        print(f"--- CRITICAL ERROR DURING TRAINING: {e} ---")
        
    finally:
        env.close()
        # Ensure we don't leave ROS hanging
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Training Finished")

if __name__ == "__main__":
    main()
