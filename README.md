# COM760 CW2 Group 6 — Multi-Robot Hazard Navigation

Two robots navigate a hazard environment. Bot 0 uses a PPO reinforcement learning policy to locate gas victims and return home. Bot 1 uses the Bug2 algorithm to navigate to a target point.

---

## Requirements

- ROS Noetic
- Gazebo 11
- Python 3
- stable-baselines3: `pip3 install stable-baselines3`

---

## Demo (standard run)

Open three terminals from the workspace root (`/home/hashie/catkin_ws`).

**Terminal 1 — Gazebo + fire oscillator**
```bash
bash src/com760cw2_group6/demo.bash
```
Builds the package, generates the world, launches Gazebo, and starts the fire oscillator. Keep this running.

**Terminal 2 — Bot 0 (PPO policy)**
```bash
python3 src/com760cw2_group6/scripts/bot0_demo.py
```
Loads the trained PPO model and runs one episode. Re-run this command as many times as needed.

**Terminal 3 — Bot 1 (Bug2 algorithm)**
```bash
python3 src/com760cw2_group6/scripts/bot1_demo.py
```
Spawns Bot 1 and navigates it toward Gas Zone 1. Re-run after each episode ends.

---

## Training (Bot 0 PPO)

```bash
cd /home/hashie/catkin_ws
bash src/com760cw2_group6/run.bash
```

In a second terminal:
```bash
python3 src/com760cw2_group6/scripts/hazard_train.py
```

Training resumes automatically from the latest checkpoint if interrupted. The model is saved to:
- `scripts/hazard_rl_model.zip`
- `scripts/hazard_rl_model_vecnormalize.pkl`

---

## Trained model backups

| Round | Location | Notes |
|-------|----------|-------|
| Round 1 | `scripts/Round1_Training/` | No convergence |
| Round 2 | `scripts/Round2_Training/` | Solved, fixed gas positions |
| Round 3 | `scripts/Round3_Training/` | **Final model**, randomised gas, 675K steps |

The active model used by `bot0_demo.py` is `scripts/hazard_rl_model.zip`.

---

## Project structure

```
scripts/
  bot0_demo.py              # Bot 0 PPO inference runner
  bot1_demo.py              # Bot 1 Bug2 runner
  fire_oscillator.py        # Moves fireballs in circles (started by demo.bash)
  hazard_env.py             # Gymnasium environment
  hazard_train.py           # PPO training loop
  generate_world.py         # Generates environment_foot.world
  Group6_Bot_Bug2_Algorithm.py
  Group6Bot_1_camera_view.py
  hazard_rl_model.zip       # Active PPO model
  hazard_rl_model_vecnormalize.pkl
launch/
  group_6_launch.launch     # Main Gazebo launch (Bot 0 only)
  bot1_demo.launch          # Bot 1 spawn + Bug2
worlds/
  environment_foot.world    # Generated world (regenerated on each run)
  spawn_config.env          # Spawn positions for both bots and gas zones
urdf/
  Group6Bot_0.urdf.xacro
  Group6Bot_1.urdf.xacro
demo.bash                   # Terminal 1 entry point
run.bash                    # Training entry point
```
