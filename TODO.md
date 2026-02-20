# TODO - Phase 2: Gripper System

## Current Status (v1.5.1)
- Navigation: COMPLETE
- Evolution: COMPLETE
- World: ADJUSTED (graspable cylinders, proportional walls)
- Manual gripper: WORKING (H/J keys)
- Wireframe mode: DOCUMENTED

## Next Steps - Autonomous Gripper

### 1. Dual Sensor System
- [ ] Add short-range sensors (0.01m) for gripper
- [ ] Keep long-range sensors (0.055m) for navigation
- [ ] Decide: Separate sensors or dual-mode?

### 2. Object Detection
- [ ] Detect cylinders in gripper range
- [ ] Calculate distance to object
- [ ] Calculate angle to object

### 3. Grasp Logic
- [ ] Approach behavior (align with object)
- [ ] Grasp trigger (when close enough)
- [ ] Hold behavior (maintain grasp)
- [ ] Release behavior (when to drop)

### 4. Force Feedback
- [ ] Detect if object is grasped
- [ ] Detect if grasp failed
- [ ] Adjust grip strength

### 5. Controller Evolution
- [ ] Multi-objective fitness (navigate + grasp)
- [ ] Extend chromosome encoding
- [ ] Test grasp success rate

## Design Decisions Needed

### Sensor Configuration
Option A: Separate gripper sensors (2 forward, short range)
Option B: Dual-mode (same sensors, different thresholds)
Option C: Dedicated object detector (single forward ray)

### Gripper URDF
Current: 2-finger parallel (prismatic joints)
Future: Add lift mechanism? (2-DOF gripper)

### Object Physics
Current: Cylinders (r=1.5cm, h=4cm, mass=20g)
Test with: Different masses? Friction? Shapes?

## Questions from Previous Session

1. Should gripper be part of navigation controller or separate?
2. How to handle "navigate to object" vs "grasp object"?
3. Fitness function: weighted sum or Pareto front?
4. Test with static cylinders first or dynamic from start?

## Files That Will Need Changes

- epuck_config.json (gripper sensors config)
- view_epuck_sim.py (gripper sensor readings)
- run_epuck_evol.py (multi-objective fitness)
- world_builder.py (maybe spawn targets in specific zones)
- New file: gripper_controller.py?
- New file: object_detector.py?

## Reference Information

Robot dimensions:
- Base radius: 3.75cm
- Gripper forward reach: 5.5cm
- Gripper width (open): 4cm
- Gripper height: 2.8-4.3cm

Cylinder dimensions:
- Radius: 1.5cm (3cm diameter)
- Height: 4cm
- Mass: 20g (pushable, graspable)

Sensor ranges:
- Navigation: 5.5cm
- Gripper (proposed): 1-2cm

# TODO - Behavior-Based Controller Integration Guide

## 1. Controller JSON Modifications

Create a new JSON file (for example: behavior_based.json) with the following
structure:

{
  "encoding": "behavior_based",
  "behaviors": [
    {
      "name": "avoid_obstacles",
      "priority": 1,
      "threshold": 0.25,
      "action": "reverse"
    },
    {
      "name": "seek_goal",
      "priority": 2,
      "sensor": "light",
      "action": "forward"
    },
    {
      "name": "explore",
      "priority": 3,
      "action": "wander"
    }
  ],
  "max_speed": 0.35
}

Key fields:
- encoding: must be "behavior_based".
- behaviors: list of behaviors with:
  * name: identifier for the behavior.
  * priority: lower number = higher priority.
  * threshold: sensor condition to trigger the behavior.
  * action: maps to existing ACTION_TABLE (forward, left, right, reverse, wander).
- max_speed: maximum wheel speed.

## 2. Simulator Code Modifications (view_epuck_sim.py)

Add a new encoding case in the function robot_controller:

elif controller_params.get("encoding") == "behavior_based":
    # Iterate behaviors in order of priority
    for behavior in sorted(controller_params["behaviors"], key=lambda b: b["priority"]):
        if behavior["name"] == "avoid_obstacles":
            if max(sensor_values) > behavior["threshold"]:
                return ACTION_TABLE[behavior["action"]]
        elif behavior["name"] == "seek_goal":
            # Example: check for goal sensor (to be implemented)
            goal_detected = False  # placeholder
            if goal_detected:
                return ACTION_TABLE[behavior["action"]]
        elif behavior["name"] == "explore":
            # Default fallback if no other behavior triggers
            return ACTION_TABLE[behavior["action"]]

Notes:
- Behaviors are evaluated in priority order.
- The first matching condition decides the action.
- Actions are mapped to the existing ACTION_TABLE defined in the config.
- You may extend sensor logic (for example, light detection) as needed.

## 3. Workflow Summary

1. Define behaviors in a new JSON file with priorities and actions.
2. Extend robot_controller to handle "behavior_based" encoding.
3. Map actions to wheel speeds using the existing ACTION_TABLE.
4. Test simulation by running:

   python view_epuck_sim.py behavior_based.json 5000 true

## 4. Advantages of Behavior-Based Control

- Modularity: easy to add or remove behaviors.
- Safety: obstacle avoidance always has highest priority.
- Flexibility: supports exploration, goal-seeking, or other tasks.
- Compatibility: integrates smoothly with existing Braitenberg and evolutionary controllers.
