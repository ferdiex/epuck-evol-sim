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