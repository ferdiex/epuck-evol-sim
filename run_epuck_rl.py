# run_epuck_rl.py
import os
import json
import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from config_loader import load_config
from world_builder import build_world_from_config


# ============================================================
# SENSOR FUNCTION
# ============================================================
def read_range_sensors(robot_id, sensor_angles_world, sensor_range):
    """
    Cast rays around the robot and return normalized proximity readings in [0, 1].
    0.0 -> nothing detected within range
    1.0 -> contact / very close obstacle
    """
    base_pos, base_ori = p.getBasePositionAndOrientation(robot_id)
    base_mat = p.getMatrixFromQuaternion(base_ori)

    # Forward and left vectors in world frame (z = 0 for planar sensors)
    fwd_vec = np.array([base_mat[0], base_mat[3], 0.0], dtype=np.float32)
    left_vec = np.array([base_mat[1], base_mat[4], 0.0], dtype=np.float32)

    readings = []
    for a in sensor_angles_world:
        direction = np.cos(a) * fwd_vec + np.sin(a) * left_vec
        start = np.array(base_pos, dtype=np.float32) + np.array([0.0, 0.0, 0.015], dtype=np.float32)
        end = start + direction * float(sensor_range)

        ray = p.rayTest(start, end)[0]
        hit_body = ray[0]
        hit_fraction = ray[2]

        if hit_body == -1 or hit_body == robot_id:
            readings.append(0.0)
        else:
            readings.append(float(1.0 - hit_fraction))

    return readings


# ============================================================
# ENVIRONMENT
# ============================================================
class EPuckEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, config_path="epuck_config.json"):
        super(EPuckEnv, self).__init__()
        self.cfg = load_config(config_path)

        # Spaces
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=(8,), dtype=np.float32)

        # Cached config values (English variable names)
        self.sensor_range = float(self.cfg["sensors"]["range"])
        self.max_speed = float(self.cfg["robot"]["max_speed"])
        self.wheel_radius = float(self.cfg["robot"]["wheel_radius"])
        self.max_motor_torque = float(self.cfg["robot"]["max_motor_torque"])
        self.timestep = float(self.cfg["world"]["timestep"])

        # Simulation stepping
        self.frame_skip = 4  # number of physics steps per env step (tune 1..8)

        # Episode control
        self.max_episode_steps = 2000
        self.episode_step = 0

        # Robot handles
        self.robot_id = None
        self.left_joint = None
        self.right_joint = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_step = 0

        if p.isConnected():
            p.disconnect()

        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, self.cfg["world"]["gravity"])
        p.setTimeStep(self.timestep)

        p.loadURDF(self.cfg["world"]["plane_urdf"])
        build_world_from_config(self.cfg)

        # Spawn away from center (simple, deterministic). You can randomize later if needed.
        start_pos = [0.4, 0.4, 0.035]
        start_ori = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        self.robot_id = p.loadURDF(self.cfg["robot"]["urdf_path"], start_pos, start_ori, useFixedBase=False)

        # Cache wheel joint indices once
        self.left_joint = None
        self.right_joint = None
        for j in range(p.getNumJoints(self.robot_id)):
            n = p.getJointInfo(self.robot_id, j)[1].decode("utf-8")
            if n == "left_wheel_joint":
                self.left_joint = j
            elif n == "right_wheel_joint":
                self.right_joint = j

        if self.left_joint is None or self.right_joint is None:
            raise RuntimeError("Wheel joints not found in URDF (left_wheel_joint / right_wheel_joint).")

        # Settle contacts
        for _ in range(10):
            p.stepSimulation()

        # Initial observation
        yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.robot_id)[1])[2]
        sensor_angles = yaw + np.linspace(-np.pi, np.pi, 8, endpoint=False)
        obs = np.array(read_range_sensors(self.robot_id, sensor_angles, self.sensor_range), dtype=np.float32)

        info = {}
        return obs, info

    def step(self, action):
        self.episode_step += 1

        # Clip and scale normalized action -> wheel linear speed (m/s)
        action = np.clip(action, -1.0, 1.0)
        wl = float(action[0]) * self.max_speed
        wr = float(action[1]) * self.max_speed

        # Apply motor control (velocity in rad/s)
        p.setJointMotorControl2(
            self.robot_id,
            self.left_joint,
            p.VELOCITY_CONTROL,
            targetVelocity=wl / self.wheel_radius,
            force=self.max_motor_torque,
        )
        p.setJointMotorControl2(
            self.robot_id,
            self.right_joint,
            p.VELOCITY_CONTROL,
            targetVelocity=wr / self.wheel_radius,
            force=self.max_motor_torque,
        )

        # Step physics multiple times per env step
        for _ in range(self.frame_skip):
            p.stepSimulation()

        # Observation
        yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.robot_id)[1])[2]
        sensor_angles = yaw + np.linspace(-np.pi, np.pi, 8, endpoint=False)
        obs = np.array(read_range_sensors(self.robot_id, sensor_angles, self.sensor_range), dtype=np.float32)

        # Reward shaping
        forward = (wl + wr) / 2.0
        obstacle = float(np.max(obs))
        turn = float(abs(wl - wr))

        # Encourage forward motion while discouraging proximity and excessive turning
        reward = 1.0 * max(forward, 0.0) * (1.0 - obstacle) - 0.05 * turn

        # Small idle penalty to avoid collapsing to zero-action policy
        if abs(forward) < 0.02:
            reward -= 0.01

        terminated = False
        truncated = self.episode_step >= self.max_episode_steps

        info = {
            "forward": float(forward),
            "obstacle": float(obstacle),
            "turn": float(turn),
            "wl": float(wl),
            "wr": float(wr),
        }

        return obs, float(reward), terminated, truncated, info


# ============================================================
# TRAINING
# ============================================================
if __name__ == "__main__":
    env = EPuckEnv()
    check_env(env, warn=True)

    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=200000)

    controller = {"encoding": "rl_policy", "policy_file": "reinforcement/epuck_rl_policy.zip"}

    os.makedirs("reinforcement", exist_ok=True)
    with open("reinforcement/epuck_rl_controller.json", "w", encoding="utf-8") as f:
        json.dump(controller, f, indent=2)

    model.save("reinforcement/epuck_rl_policy")
    print("[INFO] RL training complete, controller exported to reinforcement/")
