# Reinforcement Learning (RL) – PPO (Stable-Baselines3)

This repo includes a simple Reinforcement Learning training script for the e-puck simulator using:
- Algorithm: PPO
- Library: stable-baselines3
- Environment API: gymnasium

The RL training is run separately from the viewer, then the trained policy is loaded in the viewer via a controller JSON using encoding `rl_policy`.

## Files
- run_epuck_rl.py
  - Trains a PPO policy in DIRECT (headless) PyBullet.
  - Exports:
    - reinforcement/epuck_rl_policy.zip
    - reinforcement/epuck_rl_controller.json

- reinforcement/epuck_rl_controller.json
  - Controller file used by the viewer:
    - encoding: rl_policy
    - policy_file: reinforcement/epuck_rl_policy.zip

- reinforcement/epuck_rl_policy.zip
  - Saved Stable-Baselines3 model (PPO).

## Dependencies
Minimum:
- Python 3.x
- pybullet
- numpy
- gymnasium
- stable-baselines3

Typical install:
python -m pip install pybullet numpy gymnasium stable-baselines3

Note:
- The viewer also imports stable-baselines3 if you run an `rl_policy` controller.

## Train an RL policy (PPO)
From the repo root:

python run_epuck_rl.py

What it does (current behavior):
- Creates EPuckEnv() using epuck_config.json (default).
- Runs PyBullet in DIRECT mode.
- Trains PPO for 200000 timesteps.
- Saves:
  - reinforcement/epuck_rl_policy.zip
  - reinforcement/epuck_rl_controller.json

Output location:
- The reinforcement/ folder is created in the current working directory (repo root).
- Example path on macOS:
  /Users/fmontes/GitHub/Epuck/public/reinforcement

## Run/view the trained RL policy in the viewer
After training, run:

python view_epuck_sim.py reinforcement/epuck_rl_controller.json 5000 true

Notes:
- The viewer reads the controller JSON and loads the PPO model from policy_file.
- If the path in policy_file is relative, it is resolved from the current working directory.

## RL environment details (EPuckEnv)
Observation:
- 8 proximity readings (float32) in [0, 1]
- Ray sensor angles are:
  linspace(-pi, pi, 8, endpoint=False) + yaw
- Rays start at z = 0.015 m and have range cfg["sensors"]["range"]

Action:
- Continuous Box(2,) in [-1, 1]
- Interpreted as normalized left/right wheel commands
- Scaled by cfg["robot"]["max_speed"]
- Applied as wheel velocity control in PyBullet

Episode:
- max_episode_steps = 2000
- truncated = True when episode_step >= max_episode_steps
- terminated is currently always False (no explicit terminal condition yet)

Reward shaping (current):
- Encourages forward motion
- Penalizes proximity to obstacles and excessive turning
- Adds a small penalty for near-zero forward motion

## Current limitations (by design / TODO)
- No explicit terminal condition for collisions or reaching a goal.
- Reset spawns the robot at a fixed start position (not randomized yet).
- No explicit train/eval CLI flags (training runs on execution).
- No logging directory structure or multiple runs management yet.

These are easy to extend later if needed (e.g., add argparse flags, random starts, goal reaching, evaluation scripts).