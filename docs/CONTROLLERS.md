## Controller Configuration

### Braitenberg Controller Example

{
  "encoding": "braitenberg",
  "left_weights":  [-2.0, -1.5, -1.0, -0.5, 0.5, 1.0, 1.5, 2.0],
  "right_weights": [2.0, 1.5, 1.0, 0.5, -0.5, -1.0, -1.5, -2.0],
  "left_bias":   0.0,
  "right_bias":  0.0,
  "max_speed":   0.35
}

Parameters:
- left_weights: 8 weights connecting sensors to left wheel
- right_weights: 8 weights connecting sensors to right wheel
- left_bias: Constant forward drive (left wheel)
- right_bias: Constant forward drive (right wheel)
- max_speed: Maximum wheel speed (0.0-1.0)

Sensor mapping:
Sensor indices (0-7) arranged around robot:
     2   3   4   5
   1               6
 0                   7
      [ROBOT]

How it works:
left_wheel_speed = sum(sensor[i] * left_weight[i]) + left_bias
right_wheel_speed = sum(sensor[i] * right_weight[i]) + right_bias

Example behavior (obstacle avoidance):
- Front sensors detect wall → opposite wheel speeds → turn away
- Left sensors strong negative → right wheel speeds up → turn right
- Right sensors strong positive → left wheel speeds up → turn left

---

### Bitmask Controller Example

{
  "encoding": "bitmask",
  "chromosome": ["forward", "right", "left", "reverse", 
                 "right", "right", "left", "left"],
  "generation": 99,
  "fitness": 456.78
}

Sensor encoding (3-bit state):
- Bit 2 (MSB): Left sensors (0, 1) detect obstacle
- Bit 1: Center sensors (2, 3, 4, 5) detect obstacle
- Bit 0 (LSB): Right sensors (6, 7) detect obstacle

State lookup:

Binary  Decimal  Left      Center    Right     Action
000     0        Clear     Clear     Clear     chromosome[0] (usually "forward")
001     1        Clear     Clear     Obstacle  chromosome[1] (e.g., "left")
010     2        Clear     Obstacle  Clear     chromosome[2] (e.g., "reverse")
011     3        Clear     Obstacle  Obstacle  chromosome[3] (e.g., "left")
100     4        Obstacle  Clear     Clear     chromosome[4] (e.g., "right")
101     5        Obstacle  Clear     Obstacle  chromosome[5] (e.g., "reverse")
110     6        Obstacle  Obstacle  Clear     chromosome[6] (e.g., "right")
111     7        Obstacle  Obstacle  Obstacle  chromosome[7] (e.g., "reverse")

Available actions:
- forward: Both wheels full speed forward
- left: Left wheel slow, right wheel fast (turn left)
- right: Right wheel slow, left wheel fast (turn right)
- reverse: Both wheels backward
- stop: Both wheels stop
- wander: Asymmetric speeds for exploration

Example: If left and center sensors detect obstacles → state = 0b110 = 6 → action = chromosome[6] (e.g., "right")

---

# Controllers (Encodings) Reference
This repo uses controller JSON files. Each controller must include an `encoding` field that determines how the viewer interprets it.

The main entrypoint that loads controllers is:
- view_epuck_sim.py

## Supported encodings (as accepted by the viewer)
Stable (navigation-focused):
- bitmask
- braitenberg
- behavior_based
- leaky

Supported but not a core baseline (experimental/fine):
- braitenberg_seek
- rl_policy

Gripper-related (treat as experimental until manipulation is validated):
- braitenberg_gripper

## General JSON conventions
- Controller files are JSON.
- Minimal required fields depend on the encoding.
- Keep controller JSON paths relative to the repo root when possible (portable).

## Notes on sensors and bitmask
- The simulation uses 8 proximity sensors arranged around the robot.
- Bitmask grouping is currently legacy in code and may not match geometric left/center/right.
  If you need geometric interpretability, introduce a versioned mapping (e.g., bitmask_v2)
  rather than silently changing behavior.

## RL policies
The `rl_policy` encoding loads a trained Stable-Baselines3 model via a path, typically:
- reinforcement/epuck_rl_policy.zip

See:
- docs/RL.md