import numpy as np
import pybullet as p
import pybullet_data
import random
import sys
import time
import json

# ============================================================================
# LOAD CONFIGURATION
# ============================================================================

from config_loader import load_config, parse_config_arg
from world_builder import build_world_from_config

# Parse --config flag (or use default epuck_config.json)
CONFIG_PATH = parse_config_arg()
cfg = load_config(CONFIG_PATH)

# ============================================================================
# SIMULATION PARAMETERS (loaded from config)
# ============================================================================

NUM_SENSORS = cfg['sensors']['num_sensors']
SENSOR_RANGE = cfg['sensors']['range']
SENSOR_ANGLES = np.linspace(-np.pi, np.pi, NUM_SENSORS, endpoint=False)
WORLD_SIZE = cfg['world']['size']
OBSTACLE_THRESHOLD = cfg['sensors']['obstacle_threshold']

# ============================================================================
# VERBOSITY CONTROL (loaded from config)
# ============================================================================

VERBOSITY = cfg['output']['verbosity']
SENSOR_PRINT_FREQ = cfg['output']['sensor_print_freq']

# ============================================================================
# FEATURE FLAGS (loaded from config)
# ============================================================================

ENABLE_STUCK_DETECTION = cfg['stuck_detection']['enabled']
ENABLE_UNSTUCK = cfg['unstuck']['enabled']

# Unstuck behavior configuration
STUCK_THRESHOLD = cfg['stuck_detection']['threshold_frames']
UNSTUCK_REVERSE_FRAMES = cfg['unstuck']['reverse_frames']
UNSTUCK_TURN_FRAMES = cfg['unstuck']['turn_frames']
MOVEMENT_THRESHOLD = cfg['stuck_detection']['movement_threshold']
SENSOR_THRESHOLD = cfg['stuck_detection']['sensor_threshold']
EFFORT_THRESHOLD = cfg['stuck_detection']['effort_threshold']
GRACE_PERIOD = cfg['stuck_detection']['grace_period']
HISTORY_LENGTH = cfg['stuck_detection']['history_length']

# ============================================================================
# BITMASK CONTROLLER ACTION TABLE (loaded from config)
# ============================================================================

ACTIONS = cfg['bitmask']['actions']
ACTION_TABLE = {k: tuple(v) for k, v in cfg['bitmask']['action_table'].items()}

# ============================================================================
# CONTROLLER FUNCTIONS
# ============================================================================

def get_bitmask_state(sensor_values):
    """
    Extract 3-bit mask (left, center, right) state from sensor readings.
    Used for bitmask encoding controllers.
    """
    left = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [0,1])
    center = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [2,3,4,5])
    right = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [6,7])
    return (left << 2) | (center << 1) | (right << 0)

def robot_controller(sensor_values, controller_params):
    """
    Main robot controller - supports multiple encoding types.
    
    Args:
        sensor_values: List of 8 sensor readings [0.0-1.0]
        controller_params: Dict with encoding type and parameters
        
    Returns:
        (left_speed, right_speed): Tuple of wheel speeds [-1.0, 1.0]
    """
    if controller_params.get("encoding") in ("braitenberg", "braitenberg_gripper"):
        # Braitenberg vehicle controller (weighted sum of sensors)
        left_w = np.array(controller_params["left_weights"])
        right_w = np.array(controller_params["right_weights"])
        l = float(np.dot(left_w, sensor_values)) + controller_params.get("left_bias", 0.0)
        r = float(np.dot(right_w, sensor_values)) + controller_params.get("right_bias", 0.0)
        max_speed = controller_params.get("max_speed", cfg['robot']['max_speed'])
        l = np.clip(l, -max_speed, max_speed)
        r = np.clip(r, -max_speed, max_speed)
        return l, r
    elif controller_params.get("encoding") == "bitmask":
        # Discrete action controller (lookup table)
        idx = get_bitmask_state(sensor_values)
        action = controller_params["chromosome"][idx]
        return ACTION_TABLE[action]
    else:
        raise ValueError("Unknown controller encoding")

# ============================================================================
# SENSOR SYSTEM
# ============================================================================

def read_range_sensors(robot_id, sensor_angles_world, sensor_range):
    """
    Simulates 8 range sensors around the robot using ray casting.
    
    Args:
        robot_id: PyBullet body ID of the robot
        sensor_angles_world: Array of 8 global angles for sensor directions
        sensor_range: Maximum detection distance in meters
        
    Returns:
        List of 8 sensor readings [0.0 = no detection, 1.0 = touching obstacle]
    """
    base_pos, base_ori = p.getBasePositionAndOrientation(robot_id)
    base_mat = p.getMatrixFromQuaternion(base_ori)
    fwd_vec = np.array([base_mat[0], base_mat[3], 0])
    left_vec = np.array([base_mat[1], base_mat[4], 0])
    readings = []
    
    for si, a in enumerate(sensor_angles_world):
        direction = np.cos(a) * fwd_vec + np.sin(a) * left_vec
        start = np.array(base_pos) + np.array([0, 0, 0.015])  # Sensor height
        end = start + direction * sensor_range
        ray = p.rayTest(start, end)[0]
        hit_fraction = ray[2]
        
        if ray[0] == -1 or ray[0] == robot_id:
            # No hit or self-hit
            readings.append(0.0)
        else:
            # Hit obstacle - convert distance to proximity reading
            reading = 1.0 - hit_fraction
            readings.append(reading)
    
    return readings

# ============================================================================
# ROBOT SETUP
# ============================================================================

def random_start():
    """
    Generate random starting position and orientation within the arena.
    Includes margin from walls to avoid spawning inside obstacles.
    """
    margin = cfg['robot']['spawn_margin']
    x = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    y = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    yaw = random.uniform(-np.pi, np.pi)
    return [x, y, 0.035], p.getQuaternionFromEuler([0, 0, yaw])

def set_friction(robot_id):
    """
    Configure friction parameters for realistic wheel physics.
    High friction on wheels, low on chassis and caster.
    """
    wheel_names = ["left_wheel_joint", "right_wheel_joint"]
    wheel_friction = cfg['robot']['friction']['wheel']
    chassis_friction = cfg['robot']['friction']['chassis']
    caster_friction = cfg['robot']['friction']['caster']

    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        name = info[1].decode('utf-8')
        if name in wheel_names:
            p.changeDynamics(robot_id, j, lateralFriction=wheel_friction)
        elif "caster" in name:
            p.changeDynamics(robot_id, j, lateralFriction=caster_friction)

    p.changeDynamics(robot_id, -1, lateralFriction=chassis_friction)

# ============================================================================
# UNSTUCK BEHAVIOR
# ============================================================================

def compute_unstuck_maneuver(unstuck_phase, unstuck_frame_counter, turn_direction):
    """
    Compute wheel speeds for unstuck behavior.
    Phase 1: Reverse (back away from obstacle)
    Phase 2: Turn 180° (random direction to break symmetry)
    
    Args:
        unstuck_phase: 0=normal, 1=reversing, 2=turning
        unstuck_frame_counter: Frames spent in current unstuck phase
        turn_direction: -1 (left) or +1 (right)
        
    Returns:
        (left_speed, right_speed, new_phase, new_counter, turn_dir)
    """
    reverse_speed = cfg['unstuck']['reverse_speed']
    turn_speed = cfg['unstuck']['turn_speed']
    
    if unstuck_phase == 1:
        # Phase 1: Reverse
        if unstuck_frame_counter < UNSTUCK_REVERSE_FRAMES:
            return -reverse_speed, -reverse_speed, 1, unstuck_frame_counter + 1, turn_direction
        else:
            # Move to turning phase
            return 0.0, 0.0, 2, 0, turn_direction
    
    elif unstuck_phase == 2:
        # Phase 2: Turn 180°
        if unstuck_frame_counter < UNSTUCK_TURN_FRAMES:
            left_speed = -turn_speed * turn_direction
            right_speed = turn_speed * turn_direction
            return left_speed, right_speed, 2, unstuck_frame_counter + 1, turn_direction
        else:
            # Unstuck complete - return to normal operation
            return 0.0, 0.0, 0, 0, turn_direction
    
    else:
        # Should not reach here
        return 0.0, 0.0, 0, 0, turn_direction

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python run_epuck_sim.py controller.json [steps] [--config config.json]")
        sys.exit(0)
    filename = sys.argv[1]
    steps = int(sys.argv[2]) if len(sys.argv) > 2 else 10000

    # Load controller configuration
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)

    if data.get("encoding", "") not in ("bitmask", "braitenberg", "braitenberg_gripper"):
        print("ERROR: This visualizer supports 'bitmask', 'braitenberg', or 'braitenberg_gripper' encodings.")
        sys.exit(1)

    # Initialize PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, cfg['world']['gravity'])
    p.setTimeStep(cfg['world']['timestep'])
    p.setRealTimeSimulation(1 if cfg['world']['realtime_simulation'] else 0)
    p.loadURDF(cfg['world']['plane_urdf'])
    
    # Build world from config
    build_world_from_config(cfg)
    
    # Load robot URDF (from config)
    start_pos, start_ori = random_start()
    urdf_path = cfg['robot']['urdf_path']

    try:
        robot_id = p.loadURDF(urdf_path, start_pos, start_ori, useFixedBase=False)
        if VERBOSITY >= 1:
            print(f"[INFO] Loaded URDF: {urdf_path}")
            print(f"[INFO] Config: {CONFIG_PATH if CONFIG_PATH else 'epuck_config.json (default)'}")
    except Exception as e:
        print(f"ERROR: Could not load URDF '{urdf_path}'")
        print(f"Error details: {e}")
        p.disconnect()
        return
    
    p.resetDebugVisualizerCamera(
        cameraDistance=cfg['visualization']['camera_distance'],
        cameraYaw=cfg['visualization']['camera_yaw'],
        cameraPitch=cfg['visualization']['camera_pitch'],
        cameraTargetPosition=cfg['visualization']['camera_target']
    )

    # Find joint indices (auto-detect gripper presence)
    left_joint_index = right_joint_index = None
    left_gripper_index = right_gripper_index = None
    
    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode('utf-8')
        if name == "left_wheel_joint":
            left_joint_index = j
        elif name == "right_wheel_joint":
            right_joint_index = j
        elif name == "left_finger_joint":
            left_gripper_index = j
        elif name == "right_finger_joint":
            right_gripper_index = j
    
    if left_joint_index is None or right_joint_index is None:
        print("ERROR: Could not find wheel joint indices!")
        p.disconnect()
        return
    
    has_gripper = (left_gripper_index is not None and right_gripper_index is not None)
    
    if VERBOSITY >= 1:
        if has_gripper:
            print(f"[INFO] Gripper detected: left={left_gripper_index}, right={right_gripper_index}")
            p.enableJointForceTorqueSensor(robot_id, left_gripper_index, enableSensor=True)
            p.enableJointForceTorqueSensor(robot_id, right_gripper_index, enableSensor=True)
        else:
            print("[INFO] No gripper detected - navigation only mode")

    set_friction(robot_id)

    # Physics parameters (from config)
    wheel_radius = cfg['robot']['wheel_radius']
    max_motor_torque = cfg['robot']['max_motor_torque']

    # Control state
    fast_mode = cfg['visualization']['fast_mode_default']
    paused = False
    last_space_toggle = False
    last_p_toggle = False
    
    initial_pos = start_pos
    initial_ori = start_ori

    # Fitness tracking
    fitness_sum = 0.0
    fitness_steps = 0
    v_list = []
    d_list = []
    i_list = []
    
    # Stuck detection state
    position_history = []
    stuck_counter = 0
    
    # Unstuck behavior state
    unstuck_phase = 0  # 0=normal, 1=reversing, 2=turning
    unstuck_frame_counter = 0
    turn_direction = 0  # Will be set to -1 or +1 when unstuck triggers
    
    # Gripper state
    manual_gripper_state = 1.0  # 0=closed, 1=open

    # Print control instructions
    if VERBOSITY >= 1 and cfg['visualization']['show_controls_menu']:
        print("\n=== CONTROLS ===")
        print("SPACE: Toggle Fast/Real-time mode")
        print("G: Toggle Full Window View (PyBullet built-in)")
        print("P: Pause/Resume simulation")
        if has_gripper:
            print("H: Manual Open gripper")
            print("J: Manual Close gripper")
        print("R: Reset robot position")
        print("================\n")
        
        print(f"[INFO] Controller: {data.get('encoding', 'unknown')}")
        print(f"[INFO] Sensor range: {SENSOR_RANGE}m")
        print(f"[INFO] World type: {cfg['world']['type']}")
        print(f"[INFO] Unstuck behavior: {'ENABLED' if ENABLE_UNSTUCK else 'DISABLED'}")
        print(f"[INFO] Stuck threshold: {STUCK_THRESHOLD} frames")
        print(f"[INFO] Verbosity level: {VERBOSITY}")
        print(f"[INFO] Running for {steps} steps\n")

    try:
        for t in range(steps):
            # Check if PyBullet still connected (prevents error if GUI closed)
            if not p.isConnected():
                if VERBOSITY >= 1:
                    print("\n[INFO] PyBullet GUI closed - stopping simulation")
                break
            
            keys = p.getKeyboardEvents()
            
            # SPACE: Toggle fast mode
            space_pressed = 32 in keys and keys[32] & p.KEY_IS_DOWN
            if space_pressed and not last_space_toggle:
                fast_mode = not fast_mode
                if VERBOSITY >= 1:
                    print(f"[INFO] Fast mode {'ON' if fast_mode else 'OFF'}")
            last_space_toggle = space_pressed
            
            # P: Toggle pause
            p_pressed = ord('p') in keys and keys[ord('p')] & p.KEY_IS_DOWN
            if p_pressed and not last_p_toggle:
                paused = not paused
                if VERBOSITY >= 1:
                    print(f"[INFO] {'PAUSED' if paused else 'RESUMED'}")
            last_p_toggle = p_pressed
            
            # Manual gripper controls (H/J)
            if has_gripper:
                if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
                    manual_gripper_state = 1.0
                    if VERBOSITY >= 1:
                        print("[MANUAL] Gripper OPEN")
                
                if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
                    manual_gripper_state = 0.0
                    if VERBOSITY >= 1:
                        print("[MANUAL] Gripper CLOSE")
            
            # R: Reset robot position
            if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:
                p.resetBasePositionAndOrientation(robot_id, initial_pos, initial_ori)
                position_history.clear()
                stuck_counter = 0
                unstuck_phase = 0
                unstuck_frame_counter = 0
                if VERBOSITY >= 1:
                    print("[INFO] Robot reset to initial position")
            
            # Only simulate if not paused
            if not paused:
                robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
                yaw = p.getEulerFromQuaternion(robot_ori)[2]
                global_angles = yaw + SENSOR_ANGLES
                svals = read_range_sensors(robot_id, global_angles, SENSOR_RANGE)

                # Get controller output (Braitenberg or Bitmask)
                l_speed, r_speed = robot_controller(svals, data)
                l_speed = np.clip(l_speed, -1.0, 1.0)
                r_speed = np.clip(r_speed, -1.0, 1.0)
                
                # Position tracking for stuck detection
                if ENABLE_STUCK_DETECTION:
                    position_history.append(robot_pos[:2])
                    if len(position_history) > HISTORY_LENGTH:
                        position_history.pop(0)
                    
                    # Check if stuck
                    if len(position_history) == HISTORY_LENGTH:
                        dist_moved = np.linalg.norm(
                            np.array(position_history[-1]) - np.array(position_history[0])
                        )
                        
                        max_sensor = max(svals)
                        wheel_effort = max(abs(l_speed), abs(r_speed))
                        
                        # Only count as stuck if:
                        # 1. Barely moving
                        # 2. AND (obstacle detected OR wheels pushing hard)
                        # 3. AND not currently in unstuck maneuver
                        if (dist_moved < MOVEMENT_THRESHOLD and 
                            (max_sensor > SENSOR_THRESHOLD or wheel_effort > EFFORT_THRESHOLD) and 
                            unstuck_phase == 0):
                            if stuck_counter >= 0:  # Not in grace period
                                stuck_counter += 1
                            else:
                                stuck_counter += 1  # Counting up from grace period
                        else:
                            if stuck_counter > 0:
                                stuck_counter = 0  # Reset only if was positive
                            elif stuck_counter < 0:
                                stuck_counter += 1  # Continue grace period countdown
                
                # Unstuck behavior override (if enabled and stuck long enough)
                if ENABLE_UNSTUCK:
                    if unstuck_phase > 0:
                        # Currently executing unstuck maneuver
                        l_speed, r_speed, unstuck_phase, unstuck_frame_counter, turn_direction = \
                            compute_unstuck_maneuver(unstuck_phase, unstuck_frame_counter, turn_direction)
                        
                        if unstuck_phase == 0:
                            # Unstuck complete - add grace period to prevent immediate re-trigger
                            stuck_counter = -GRACE_PERIOD
                            position_history.clear()
                            if VERBOSITY >= 2:
                                print(f"[UNSTUCK] Maneuver complete at t={t}, grace period active")
                    
                    elif stuck_counter > STUCK_THRESHOLD:
                        # Trigger unstuck behavior
                        if VERBOSITY >= 2:
                            print(f"[UNSTUCK] Triggered at t={t}, pos=({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
                            print(f"[UNSTUCK] Reason: moved {dist_moved:.3f}m, sensor_max={max_sensor:.2f}, effort={wheel_effort:.2f}")
                            print(f"[UNSTUCK] Turn direction: {'LEFT' if turn_direction < 0 else 'RIGHT'}")
                        unstuck_phase = 1
                        unstuck_frame_counter = 0
                        turn_direction = random.choice([-1, 1])  # Random turn direction
                
                # Gripper control (manual only for now)
                if has_gripper:
                    if manual_gripper_state < 0.5:
                        # CLOSE
                        p.setJointMotorControl2(robot_id, left_gripper_index, 
                                               p.POSITION_CONTROL, targetPosition=-0.020, force=10)
                        p.setJointMotorControl2(robot_id, right_gripper_index, 
                                               p.POSITION_CONTROL, targetPosition=0.020, force=10)
                    else:
                        # OPEN
                        p.setJointMotorControl2(robot_id, left_gripper_index, 
                                               p.POSITION_CONTROL, targetPosition=0.020, force=10)
                        p.setJointMotorControl2(robot_id, right_gripper_index, 
                                               p.POSITION_CONTROL, targetPosition=-0.020, force=10)
                
                # Debug output - sensor readings
                if VERBOSITY >= 3 and t % SENSOR_PRINT_FREQ == 0:
                    sensor_str = "[" + ", ".join([f"{s:.2f}" for s in svals]) + "]"
                    phase_str = f" [UNSTUCK:{unstuck_phase}]" if unstuck_phase > 0 else ""
                    grace_str = f" [GRACE:{abs(stuck_counter)}]" if stuck_counter < 0 else ""
                    print(f"[t={t:5d}] Sensors: {sensor_str} | L/R: {l_speed:+.2f}/{r_speed:+.2f} | Pos: ({robot_pos[0]:+.2f}, {robot_pos[1]:+.2f}){phase_str}{grace_str}")
                
                # Stuck warning (full debug mode only)
                if VERBOSITY >= 4 and stuck_counter > 0 and stuck_counter % 100 == 0 and unstuck_phase == 0:
                    if len(position_history) >= HISTORY_LENGTH:
                        dist = np.linalg.norm(np.array(position_history[-1]) - np.array(position_history[0]))
                        max_s = max(svals)
                        effort = max(abs(l_speed), abs(r_speed))
                        print(f"[WARNING] Robot appears STUCK! (counter={stuck_counter}, moved={dist:.3f}m, sensor={max_s:.2f}, effort={effort:.2f})")
                
                # Fitness calculation (Nolfi-style: velocity, straight movement, obstacle avoidance)
                fitness_cfg = cfg['fitness']['nolfi']
                max_speed = fitness_cfg['max_speed']
                v_left = l_speed
                v_right = r_speed
                V = (abs(v_left) + abs(v_right)) / (2.0 * max_speed)
                D = abs(v_left - v_right) / (2.0 * max_speed)
                I = max(svals) if len(svals) > 0 else 0.0       
                
                v_list.append(V)
                d_list.append(D)
                i_list.append(I) 
                
                fitness_timestep = V * (1 - np.sqrt(D)) * (1 - I)
                fitness_sum += fitness_timestep
                fitness_steps += 1    

                # Apply wheel velocities
                wl = l_speed / wheel_radius
                wr = r_speed / wheel_radius
                p.setJointMotorControl2(robot_id, left_joint_index, p.VELOCITY_CONTROL, 
                                       targetVelocity=wl, force=max_motor_torque)
                p.setJointMotorControl2(robot_id, right_joint_index, p.VELOCITY_CONTROL, 
                                       targetVelocity=wr, force=max_motor_torque)
                p.stepSimulation()
                
                if not fast_mode:
                    time.sleep(0.0003)
            else:
                # Paused - just wait
                time.sleep(0.01)
    
    except KeyboardInterrupt:
        if VERBOSITY >= 1:
            print("\n[INFO] Interrupted by user (Ctrl+C)")
    except p.error as e:
        if VERBOSITY >= 1:
            print(f"\n[INFO] PyBullet error: {e}")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Only disconnect if still connected
        if p.isConnected():
            p.disconnect()
        if VERBOSITY >= 1:
            print("\n=== Simulation finished ===")

    # Print final statistics
    if fitness_steps > 0 and VERBOSITY >= 1:
        avg_fitness = fitness_sum / fitness_steps
        print(f"\n=== RESULTS ===")
        print(f"Total steps: {fitness_steps}")
        print(f"Nolfi-style fitness (mean): {avg_fitness:.4f}")
        print(f"Mean V (velocity): {np.mean(v_list):.3f}")
        print(f"Mean D (turn deviation): {np.mean(d_list):.3f}")
        print(f"Mean I (obstacle proximity): {np.mean(i_list):.3f}")
        print(f"Final stuck counter: {stuck_counter}")
        print("================\n")

if __name__ == "__main__":
    main()