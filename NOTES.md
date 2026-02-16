# Project Notes - E-puck Simulator

## Design Philosophy

### Evolution-First Approach
- Controllers are SIMPLE (reactive, no memory)
- Getting stuck is EXPECTED (evolution solves it)
- Unstuck behavior DISABLED for evolution (natural selection)
- Manual testing: unstuck enabled for demos

### Why Simple Controllers?
- Braitenberg: sensor-motor direct connections
- Bitmask: 8-rule lookup table (3-bit sensor state)
- Evolution discovers complex behaviors from simple rules
- No neural networks yet (Phase 3)

## Key Decisions Made

### v1.0.0 (2026-02-12)
- Braitenberg controller implemented
- Nolfi fitness function
- Basic walls world
- Manual gripper (H/J keys)

### v1.5.0 (2026-02-13)
- Evolution system complete
- Plotting separated from evolution (stability)
- Dual viewers (CLI + GUI)
- Config centralized
- Logs timestamped (never overwrite)

### v1.5.1 (2026-02-16)
- Wall thickness: 5cm -> 2cm (thinner)
- Wall height: 30cm -> 8cm (proportional)
- Cylinders: r=5cm -> 1.5cm (graspable)
- Cylinders: h=20cm -> 4cm (visible but not towering)
- Cylinders: mass=0 -> 20g (pushable)
- Wireframe control documented

## Known Issues

### Stuck Detection
- False positives in open space with low bias
- Workaround: Increase bias or disable unstuck
- Proper fix planned for v1.6.0

### Gripper
- Manual only (H/J keys)
- No autonomous grasping yet
- Phase 2 work

## Configuration Presets

### Easy Navigation
count: 3, radius: 0.01, spawn_margin: 0.3

### Medium Navigation
count: 5, radius: 0.015, spawn_margin: 0.25

### Hard Navigation
count: 8, radius: 0.018, spawn_margin: 0.2

## Conversation History

Session 1 (2026-02-12 to 2026-02-16):
- Built complete evolution system
- Dual viewers
- World adjustments
- Documentation complete
- ~80% context used
- Ready for new conversation