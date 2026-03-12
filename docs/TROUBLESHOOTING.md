## Known Issues & Limitations

### 1. False Stuck Detection in Open Space

Issue: Robot can trigger stuck detection when moving slowly with no obstacles nearby.

Symptoms:
[WARNING] Robot appears STUCK! (counter=200, moved=0.004m in 50 frames)
Sensors: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]

Root Causes:
- Low bias values (< 0.15) → robot barely overcomes friction
- Stuck detection doesn't distinguish between "pushing against wall" and "moving slowly in open space"
- PyBullet friction creates realistic "stiction" below certain speeds

Workarounds:
1. Increase bias in controller: "left_bias": 0.2, "right_bias": 0.2
2. Increase stuck threshold: STUCK_THRESHOLD = 300
3. Disable unstuck for evolution: ENABLE_UNSTUCK = False

Proper fix (TODO):
# Only trigger if obstacle present OR wheels spinning hard
if dist_moved < 0.05 and (max(sensors) > 0.3 or abs(wheel_speed) > 0.2):
    stuck_counter += 1

Status: KNOWN ISSUE - Workarounds available, proper fix planned for v1.6.0

### 2. Corner Traps (Mitigated by Evolution)

Issue: Symmetric sensor readings in corners can trap reactive controllers.

Status: BY DESIGN - Evolution should discover escape strategies.

Solutions:
- Unstuck behavior (for testing/demos)
- Evolved asymmetric controllers (production)

### 3. Sensor Dead Zones

Issue: With 8 sensors, gaps between sensor cones can miss small obstacles.

Impact: Low (walls are large, gaps are small)

Future: 16 sensors or wider cones

### 4. Gripper Not Yet Autonomous

Status: Manual control (H/J keys) works, but no autonomous grasping.

Next steps:
1. Dual sensor system (navigation 5.5cm, gripper 1cm)
2. Object detection logic
3. Grasp force feedback
4. Evolve gripper controllers

## Troubleshooting

### Robot doesn't move
- Check max_speed in controller (should be 0.3-0.5)
- Check left_bias and right_bias (both 0.0 is very conservative)
- Check sensor readings (all 0.0 means no obstacles to react to)

### Robot stuck in corner forever
- Enable unstuck behavior: ENABLE_UNSTUCK = True
- Lower stuck threshold: STUCK_THRESHOLD = 150
- Check sensor range (should be 0.055, not 0.01)

### Simulation too slow
- Press SPACE for fast mode
- Reduce DEBUG_PRINT_FREQ (less console output)
- Set VERBOSITY = 0 in config

### Gripper doesn't respond
- Check if gripper URDF loaded (should see [INFO] Gripper detected)
- Try H/J keys multiple times (finger movement is slow)
- Check PyBullet GUI - fingers should be visible

### "Could not load URDF" error
- Check robots/ folder exists in same directory
- Check URDF file names match exactly (epuck_sim.urdf, epuck_gripper.urdf)
- Try absolute path in config

### Evolution log overwrites previous runs
Fixed in v1.5.0! Logs now auto-timestamp: evolution_20260213_143052.log

### macOS: PyBullet crashes when closing window
Fixed in v1.5.0! Matplotlib removed from evolution. Use separate plotting:
python plot_fitness.py best_chromosomes/evolution_*.log

### Unstuck triggers in open space (no obstacles)
Known issue - see section above. Increase controller bias or disable unstuck for evolution.

---