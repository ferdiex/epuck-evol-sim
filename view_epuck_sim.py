#!/usr/bin/env python3
"""
ePuck Controller Viewer - GUI application for testing evolved controllers.

Usage:
    python view_epuck_sim.py                           # Launch GUI
    python view_epuck_sim.py controller.json 5000 true # Command-line mode
"""

import sys
import os
import time
import json
import random
import subprocess
import tkinter as tk
from tkinter import filedialog, messagebox

import numpy as np
import pybullet as p
import pybullet_data

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

VERBOSITY = cfg['output']['verbosity']

# ============================================================================
# BITMASK CONTROLLER ACTION TABLE (loaded from config)
# ============================================================================

ACTIONS = cfg['bitmask']['actions']
ACTION_TABLE = {k: tuple(v) for k, v in cfg['bitmask']['action_table'].items()}

# ============================================================================
# CONTROLLER FUNCTIONS
# ============================================================================

def get_bitmask_state(sensor_values):
    """Extract 3-bit mask (left, center, right) state from sensor readings."""
    left = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [0, 1])
    center = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [2, 3, 4, 5])
    right = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [6, 7])
    return (left << 2) | (center << 1) | (right << 0)

def robot_controller(sensor_values, controller_params):
    """Dispatch to Braitenberg or bitmask controller based on encoding field."""
    if controller_params.get("encoding") in ("braitenberg", "braitenberg_gripper"):
        left_w = np.array(controller_params["left_weights"])
        right_w = np.array(controller_params["right_weights"])
        l = float(np.dot(left_w, sensor_values)) + controller_params.get("left_bias", 0.0)
        r = float(np.dot(right_w, sensor_values)) + controller_params.get("right_bias", 0.0)
        max_speed = controller_params.get("max_speed", cfg['robot']['max_speed'])
        l = np.clip(l, -max_speed, max_speed)
        r = np.clip(r, -max_speed, max_speed)
        return l, r
    elif controller_params.get("encoding") == "bitmask":
        idx = get_bitmask_state(sensor_values)
        action = controller_params["chromosome"][idx]
        return ACTION_TABLE[action]
    else:
        raise ValueError("Unknown controller encoding")

# ============================================================================
# SENSOR SYSTEM
# ============================================================================

def read_range_sensors(robot_id, sensor_angles_world, sensor_range):
    """Cast rays around the robot and return normalized proximity readings."""
    base_pos, base_ori = p.getBasePositionAndOrientation(robot_id)
    base_mat = p.getMatrixFromQuaternion(base_ori)
    fwd_vec = np.array([base_mat[0], base_mat[3], 0])
    left_vec = np.array([base_mat[1], base_mat[4], 0])
    readings = []
    for si, a in enumerate(sensor_angles_world):
        direction = np.cos(a) * fwd_vec + np.sin(a) * left_vec
        start = np.array(base_pos) + np.array([0, 0, 0.015])
        end = start + direction * sensor_range
        ray = p.rayTest(start, end)[0]
        hit_fraction = ray[2]
        if ray[0] == -1 or ray[0] == robot_id:
            readings.append(0.0)
        else:
            readings.append(1.0 - hit_fraction)
    return readings

# ============================================================================
# ROBOT SETUP
# ============================================================================

def random_start():
    """Spawn the robot randomly inside the arena, away from walls."""
    margin = cfg['robot']['spawn_margin']
    x = random.uniform(-WORLD_SIZE/2 + margin, WORLD_SIZE/2 - margin)
    y = random.uniform(-WORLD_SIZE/2 + margin, WORLD_SIZE/2 - margin)
    yaw = random.uniform(-np.pi, np.pi)
    return [x, y, 0.035], p.getQuaternionFromEuler([0, 0, yaw])

def set_friction(robot_id):
    """Set high friction on wheels and low friction on base and caster."""
    wheel_names = ["left_wheel_joint", "right_wheel_joint"]
    wheel_friction = cfg['robot']['friction']['wheel']
    chassis_friction = cfg['robot']['friction']['chassis']
    caster_friction = cfg['robot']['friction']['caster']

    # Set friction for all links
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        name = info[1].decode('utf-8')
        if name in wheel_names:
            p.changeDynamics(robot_id, j, lateralFriction=wheel_friction)
        elif "caster" in name:
            p.changeDynamics(robot_id, j, lateralFriction=caster_friction)

    # Base link (chassis)
    p.changeDynamics(robot_id, -1, lateralFriction=chassis_friction)

# ============================================================================
# MAIN SIMULATION
# ============================================================================

def run_simulation(controller_file, steps, use_gui=True):
    """Run a single simulation episode using the given controller JSON file."""
    with open(controller_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data.get("encoding", "") not in ("bitmask", "braitenberg", "braitenberg_gripper"):
        messagebox.showerror("Error", "Invalid encoding. Must be 'bitmask', 'braitenberg', or 'braitenberg_gripper'.")
        return

    p.connect(p.GUI if use_gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, cfg['world']['gravity'])
    p.setTimeStep(cfg['world']['timestep'])
    p.setRealTimeSimulation(1 if cfg['world']['realtime_simulation'] else 0)
    p.loadURDF(cfg['world']['plane_urdf'])
    
    # Build world from config
    build_world_from_config(cfg)
    
    start_pos, start_ori = random_start()
    robot_id = p.loadURDF(cfg['robot']['urdf_path'], start_pos, start_ori, useFixedBase=False)
    
    p.resetDebugVisualizerCamera(
        cameraDistance=cfg['visualization']['camera_distance'],
        cameraYaw=cfg['visualization']['camera_yaw'],
        cameraPitch=cfg['visualization']['camera_pitch'],
        cameraTargetPosition=cfg['visualization']['camera_target']
    )
    
    # Find joint indices (wheels + gripper auto-detection)
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
        messagebox.showerror("Error", "Wheel joints not found.")
        p.disconnect()
        return

    # Check if gripper detected
    has_gripper = (left_gripper_index is not None and right_gripper_index is not None)
    
    if VERBOSITY >= 1:
        print(f"\n[INFO] Running simulation with controller: {controller_file}")
        print(f"[INFO] Config: {CONFIG_PATH if CONFIG_PATH else 'epuck_config.json (default)'}")
        print(f"[INFO] Sensor range: {SENSOR_RANGE}m")
        print(f"[INFO] World type: {cfg['world']['type']}")
        print(f"[INFO] Steps: {steps}")
        if has_gripper:
            print(f"[INFO] Gripper detected: left={left_gripper_index}, right={right_gripper_index}")
        else:
            print("[INFO] No gripper detected - navigation only mode")
        
        print("\n=== CONTROLS ===")
        print("SPACE: Toggle Fast/Real-time mode")
        print("P: Pause/Resume simulation")
        print("R: Reset robot position")
        if has_gripper:
            print("H: Open gripper")
            print("J: Close gripper")
        print("G: Toggle Full Window (PyBullet built-in)")
        print("================\n")

    # Set friction for robot
    set_friction(robot_id)

    wheel_radius = cfg['robot']['wheel_radius']
    max_motor_torque = cfg['robot']['max_motor_torque']

    fast_mode = cfg['visualization']['fast_mode_default']
    paused = False
    last_space_toggle = False
    last_p_toggle = False
    
    # Gripper state
    manual_gripper_state = 1.0  # 0=closed, 1=open
    
    fitness_sum = 0.0
    fitness_steps = 0
    v_list, d_list, i_list = [], [], []

    try:
        for t in range(steps):
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
            
            # R: Reset robot position
            if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:
                p.resetBasePositionAndOrientation(robot_id, start_pos, start_ori)
                if VERBOSITY >= 1:
                    print("[INFO] Robot reset to initial position")
            
            # H/J: Gripper controls (only if gripper present)
            if has_gripper:
                if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
                    manual_gripper_state = 1.0  # Open
                    if VERBOSITY >= 1:
                        print("[MANUAL] Gripper OPEN")
                
                if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
                    manual_gripper_state = 0.0  # Close
                    if VERBOSITY >= 1:
                        print("[MANUAL] Gripper CLOSE")
            
            # Only simulate if not paused
            if not paused:
                robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
                yaw = p.getEulerFromQuaternion(robot_ori)[2]
                global_angles = yaw + SENSOR_ANGLES
                svals = read_range_sensors(robot_id, global_angles, SENSOR_RANGE)
                     
                l_speed, r_speed = robot_controller(svals, data)
                l_speed = np.clip(l_speed, -1.0, 1.0)
                r_speed = np.clip(r_speed, -1.0, 1.0)

                # Nolfi-style fitness calculation
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

                # Gripper control (manual only)
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

                wl = l_speed / wheel_radius
                wr = r_speed / wheel_radius
                p.setJointMotorControl2(
                    robot_id, left_joint_index,
                    p.VELOCITY_CONTROL, targetVelocity=wl,
                    force=max_motor_torque
                )
                p.setJointMotorControl2(
                    robot_id, right_joint_index,
                    p.VELOCITY_CONTROL, targetVelocity=wr,
                    force=max_motor_torque
                )
                p.stepSimulation()
                if not fast_mode:
                    time.sleep(0.0003)
            else:
                # Paused - just wait
                time.sleep(0.01)

    finally:
        p.disconnect()

        if fitness_steps > 0:
            avg_fitness = fitness_sum / fitness_steps
            result_text = (
                f"Average fitness: {avg_fitness:.4f}\n"
                f"Mean V: {np.mean(v_list):.3f}\n"
                f"Mean D: {np.mean(d_list):.3f}\n"
                f"Mean I: {np.mean(i_list):.3f}"
            )

            with open("fitness_results.json", "w", encoding="utf-8") as jsonfile:
                json.dump({
                    'avg_fitness': avg_fitness,
                    'mean_V': float(np.mean(v_list)),
                    'mean_D': float(np.mean(d_list)),
                    'mean_I': float(np.mean(i_list))
                }, jsonfile, indent=4)
            
            if VERBOSITY >= 1:
                print(f"\n=== RESULTS ===")
                print(f"Average fitness: {avg_fitness:.4f}")
                print(f"Mean V: {np.mean(v_list):.3f}")
                print(f"Mean D: {np.mean(d_list):.3f}")
                print(f"Mean I: {np.mean(i_list):.3f}")
                print("================\n")
        else:
            result_text = "No steps evaluated for fitness."

        # Show popup only if we are in the main GUI
        if len(sys.argv) == 1:
            messagebox.showinfo("Results", result_text)

        # Safe exit depending on mode
        if use_gui:
            os._exit(0)  # Avoid crash on macOS with PyBullet GUI
        else:
            sys.exit(0)  # Allow popup to be shown in DIRECT mode

# ============================================================================
# TKINTER GUI ENTRYPOINT
# ============================================================================

if len(sys.argv) > 1:
    # Launched as: python view_epuck_sim.py controller.json steps use_gui [--config config.json]
    controller_file = sys.argv[1]
    steps = int(sys.argv[2])
    use_gui = sys.argv[3].lower() == "true"
    run_simulation(controller_file, steps, use_gui)
    sys.exit(0)

# Create GUI
root = tk.Tk()
root.title("ePuck Controller Viewer")

# Define consistent fonts
FONT_LABEL = ("Arial", 14)
FONT_ENTRY = ("Arial", 14)
FONT_BUTTON = ("Arial", 14, "bold")
FONT_INFO = ("Arial", 13)
FONT_CONTROLS = ("Arial", 12, "italic")

file_label = tk.Label(root, text="Controller JSON file:", font=FONT_LABEL)
file_label.grid(row=0, column=0, padx=5, pady=5, sticky='e')
file_entry = tk.Entry(root, width=40, font=FONT_ENTRY)
file_entry.grid(row=0, column=1, padx=5, pady=5)

def browse_file():
    """Open a file dialog to select a JSON controller file."""
    filename = filedialog.askopenfilename(
        title="Select JSON controller file",
        filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
    )
    if filename:
        file_entry.delete(0, tk.END)
        file_entry.insert(0, filename)

browse_btn = tk.Button(root, text="Browse", command=browse_file, font=FONT_BUTTON)
browse_btn.grid(row=0, column=2, padx=5, pady=5)

steps_label = tk.Label(root, text="Number of steps:", font=FONT_LABEL)
steps_label.grid(row=1, column=0, padx=5, pady=5, sticky='e')
steps_entry = tk.Entry(root, width=10, font=FONT_ENTRY)
steps_entry.insert(0, "5000")
steps_entry.grid(row=1, column=1, padx=5, pady=5, sticky='w')

mode_var = tk.BooleanVar(value=True)
mode_check = tk.Checkbutton(root, text="Use GUI (uncheck for DIRECT mode)", variable=mode_var, font=FONT_LABEL)
mode_check.grid(row=2, column=0, columnspan=3, pady=5)

info_label = tk.Label(
    root, 
    text=f"Config: {CONFIG_PATH if CONFIG_PATH else 'epuck_config.json (default)'}\n"
         f"Sensor range: {SENSOR_RANGE}m | World: {cfg['world']['type']}", 
    fg="blue",
    font=FONT_INFO
)
info_label.grid(row=3, column=0, columnspan=3, pady=5)

controls_label = tk.Label(
    root,
    text="Controls: SPACE (fast) | P (pause) | R (reset) | H/J (gripper) | G (fullscreen)",
    fg="gray",
    font=FONT_CONTROLS
)
controls_label.grid(row=4, column=0, columnspan=3, pady=5)

def run_clicked():
    """Callback for the 'Run Simulation' button."""
    controller_file = file_entry.get()
    steps = steps_entry.get()
    use_gui = mode_var.get()
    
    if not controller_file:
        messagebox.showerror("Error", "Please select a JSON controller file")
        return
    
    if not os.path.exists(controller_file):
        messagebox.showerror("Error", f"File not found: {controller_file}")
        return
    
    try:
        steps_int = int(steps)
        if steps_int <= 0:
            messagebox.showerror("Error", "Steps must be a positive integer")
            return
    except ValueError:
        messagebox.showerror("Error", "Steps must be a valid number")
        return

    # Disable the button while the simulation is running
    run_btn.config(state=tk.DISABLED)
    status_label.config(text="Running simulation...", fg="orange")
    root.update()

    # Launch the simulation in a separate process
    args = ["python", "view_epuck_sim.py", controller_file, str(steps_int), str(use_gui)]
    if CONFIG_PATH:
        args.extend(["--config", CONFIG_PATH])
    proc = subprocess.Popen(args)

    # Periodically check if the process has finished
    def check_process():
        if proc.poll() is None:
            root.after(1000, check_process)  # Check again in 1 second
        else:
            # Re-enable the button when the simulation finishes
            run_btn.config(state=tk.NORMAL)
            status_label.config(text="Ready", fg="green")

            # Read results from JSON and show a popup
            try:
                with open("fitness_results.json", "r", encoding="utf-8") as f:
                    data = json.load(f)
                messagebox.showinfo(
                    "Simulation Results",
                    f"Average fitness: {data['avg_fitness']:.4f}\n"
                    f"Mean V: {data['mean_V']:.3f}\n"
                    f"Mean D: {data['mean_D']:.3f}\n"
                    f"Mean I: {data['mean_I']:.3f}"
                )
            except Exception as e:
                messagebox.showerror("Error", f"Could not read results: {e}")

    check_process()

run_btn = tk.Button(root, text="Run Simulation", command=run_clicked, bg="lightgreen", font=("Arial", 12, "bold"))
run_btn.grid(row=5, column=0, columnspan=3, pady=10)

status_label = tk.Label(root, text="Ready", fg="green", font=FONT_INFO)
status_label.grid(row=6, column=0, columnspan=3, pady=5)

# Center window on screen
root.update_idletasks()
width = root.winfo_width()
height = root.winfo_height()
x = (root.winfo_screenwidth() // 2) - (width // 2)
y = (root.winfo_screenheight() // 2) - (height // 2)
root.geometry(f'{width}x{height}+{x}+{y}')

root.mainloop()