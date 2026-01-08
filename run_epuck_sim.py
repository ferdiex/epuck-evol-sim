from tkinter import filedialog, messagebox
import pybullet as p
import pybullet_data
import tkinter as tk
import numpy as np
import threading
import subprocess
import random
import csv
import sys
import time
import json
import os

NUM_SENSORS = 8
SENSOR_RANGE = 0.01
SENSOR_ANGLES = np.linspace(-np.pi, np.pi, NUM_SENSORS, endpoint=False)
WORLD_SIZE = 1.5
OBSTACLE_THRESHOLD = 0.25

ACTIONS = [
    "forward", "left", "right", "reverse", "stop", "wander"
]
ACTION_TABLE = {
    "forward": (1.0, 1.0),
    "left":    (0.2, 1.0),
    "right":   (1.0, 0.2),
    "reverse": (-0.5, -0.5),
    "stop":    (0.0, 0.0),
    "wander":  (0.6, 0.35),
}

def get_bitmask_state(sensor_values):
    """Extract 3-bit mask (left, center, right) state from sensor readings."""
    left = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [0,1])
    center = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [2,3,4,5])
    right = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [6,7])
    return (left << 2) | (center << 1) | (right << 0)

def robot_controller(sensor_values, controller_params):
    if controller_params.get("encoding") == "braitenberg":
        left_w = np.array(controller_params["left_weights"])
        right_w = np.array(controller_params["right_weights"])
        l = float(np.dot(left_w, sensor_values)) + controller_params.get("left_bias", 0.0)
        r = float(np.dot(right_w, sensor_values)) + controller_params.get("right_bias", 0.0)
        max_speed = controller_params.get("max_speed", 0.35)
        l = np.clip(l, 0.0, max_speed)
        r = np.clip(r, 0.0, max_speed)
        # print(f"[DEBUG] Braitenberg | L: {l:.2f}, R: {r:.2f} | Sensors: {np.round(sensor_values,2)}")
        return l, r
    elif controller_params.get("encoding") == "bitmask":
        idx = get_bitmask_state(sensor_values)
        action = controller_params["chromosome"][idx]
        # print(f"[DEBUG] Bitmask: {idx:03b} | Action: {action} | Sensors: {np.round(sensor_values,2)}")
        return ACTION_TABLE[action]
    else:
        raise ValueError("Unknown controller encoding")

'''
def add_world():
    """Sets up box arena and obstacles."""
    half = WORLD_SIZE / 2.0
    height = 0.06
    thick = 0.03
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2], rgbaColor=[.8,.8,.8,1])
    p.createMultiBody(0, wall_col, wall_vis, [0, half, height/2])
    p.createMultiBody(0, wall_col, wall_vis, [0, -half, height/2])
    wall_col_x = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2])
    wall_vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2], rgbaColor=[.8,.8,.8,1])
    p.createMultiBody(0, wall_col_x, wall_vis_x, [half, 0, height/2])
    p.createMultiBody(0, wall_col_x, wall_vis_x, [-half, 0, height/2])
    
    # Obstacles
    ob_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.03, .03, .035])
    ob_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[.03, .03, .035], rgbaColor=[1.0,0.15,0.15,1])
    for x, y in [(-0.30, 0.22), (0.38, 0.08), (0.0, -0.38)]:
        p.createMultiBody(0, ob_col, ob_vis, [x, y, 0.035])
'''

def add_world():
    """Sets up box arena, obstacles, and a recharge area."""
    half = WORLD_SIZE / 2.0
    height = 0.06
    thick = 0.03

    # Walls (Y)
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2], rgbaColor=[.8, .8, .8, 1])
    p.createMultiBody(0, wall_col, wall_vis, [0, half, height/2])
    p.createMultiBody(0, wall_col, wall_vis, [0, -half, height/2])

    # Walls (X)
    wall_col_x = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2])
    wall_vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2], rgbaColor=[.8, .8, .8, 1])
    p.createMultiBody(0, wall_col_x, wall_vis_x, [half, 0, height/2])
    p.createMultiBody(0, wall_col_x, wall_vis_x, [-half, 0, height/2])

    # Obstacles
    ob_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.03, .03, .035])
    ob_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[.03, .03, .035], rgbaColor=[1.0, 0.15, 0.15, 1])
    for x, y in [(-0.30, 0.22), (0.38, 0.08), (0.0, -0.38)]:
        p.createMultiBody(0, ob_col, ob_vis, [x, y, 0.035])

    # Recharge area (blue cylinder, no collision for now, but collision can be left in for future use)
    recharge_center = (0.5, -0.5)
    recharge_radius = 0.07
    recharge_thickness = 0.005
    recharge_vis = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=recharge_radius,
        length=recharge_thickness,
        rgbaColor=[0.1, 0.2, 1, 0.8]
    )
    recharge_col = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=recharge_radius,
        height=recharge_thickness
    )
    p.createMultiBody(
        0,  # mass = 0 (static)
        recharge_col,
        recharge_vis,
        [recharge_center[0], recharge_center[1], recharge_thickness / 2]
    )

def read_range_sensors(robot_id, sensor_angles_world, sensor_range):
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
            readings.append(1.0-hit_fraction)
        # Debug print (optional)
        # print(f"Ray{si}: {'hit self' if ray[0]==robot_id else ('open space' if ray[0]==-1 else ray[0])}")
    return readings

def random_start():
    margin = 0.15
    x = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    y = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    yaw = random.uniform(-np.pi, np.pi)
    return [x,y,0.035], p.getQuaternionFromEuler([0, 0, yaw])

def set_friction(robot_id):
    # HIGH friction on wheels, LOW on base & caster
    wheel_names = ["left_wheel_joint", "right_wheel_joint"]
    caster_name = "caster"  # as a link, not a joint!
    wheel_friction = 1.5
    chassis_friction = 0.01
    caster_friction = 0.05

    # Set friction for all links
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        name = info[1].decode('utf-8')
        # Set HIGH friction for wheels
        if name in wheel_names:
            p.changeDynamics(robot_id, j, lateralFriction=wheel_friction)
        # Set LOW friction for caster
        elif "caster" in name:
            p.changeDynamics(robot_id, j, lateralFriction=caster_friction)

    # Set LOW friction for chassis (base link, typically -1)
    p.changeDynamics(robot_id, -1, lateralFriction=chassis_friction)


def run_simulation(controller_file, steps, use_gui=True):
    with open(controller_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data.get("encoding", "") not in ("bitmask", "braitenberg"):
        messagebox.showerror("Error", "Encoding inválido. Debe ser 'bitmask' o 'braitenberg'.")
        return

    p.connect(p.GUI if use_gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    add_world()
    start_pos, start_ori = random_start()
    robot_id = p.loadURDF("robots/epuck_sim.urdf", start_pos, start_ori, useFixedBase=False)
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=45, cameraPitch=-35, cameraTargetPosition=[0, 0, 0.03])
    
    # Find joint indices (optionally print joint names for diagnostics)
    left_joint_index = right_joint_index = None
    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode('utf-8')
        if name == "left_wheel_joint":
            left_joint_index = j
        elif name == "right_wheel_joint":
            right_joint_index = j
    if left_joint_index is None or right_joint_index is None:
        messagebox.showerror("Error", "No se encontraron las ruedas.")
        p.disconnect()
        return

    # --- SET FRICTION as needed ---
    set_friction(robot_id)

    wheel_radius = 0.02
    max_motor_torque = 0.9

    fast_mode = False
    last_space_toggle = False
    
    fitness_sum = 0.0
    fitness_steps = 0
    v_list, d_list, i_list = [], [], []

    # DEBUG
    v_list = []
    d_list = []
    i_list = []

    try:
        for t in range(steps):
        
            keys = p.getKeyboardEvents()
            space_pressed = 32 in keys and keys[32] & p.KEY_IS_DOWN
            if space_pressed and not last_space_toggle:
                fast_mode = not fast_mode
                print("[INFO] Fast mode ON." if fast_mode else "[INFO] Real-time mode ON.")
            last_space_toggle = space_pressed
            
            robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
            yaw = p.getEulerFromQuaternion(robot_ori)[2]
            global_angles = yaw + SENSOR_ANGLES
            svals = read_range_sensors(robot_id, global_angles, SENSOR_RANGE)
            
            '''
            # DEBUG
            if t < 40:
                print(f"Sensors at start: {np.round(svals, 3)}")
            '''
                 
            l_speed, r_speed = robot_controller(svals, data)
            l_speed = np.clip(l_speed, -1.0, 1.0)
            r_speed = np.clip(r_speed, -1.0, 1.0)

            # --- Nolfi et al. fitness calculation ---
            max_speed = 1.0  # Adjust if your actual max controller output is different!
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

            wl = l_speed / wheel_radius
            wr = r_speed / wheel_radius
            p.setJointMotorControl2(robot_id, left_joint_index, p.VELOCITY_CONTROL, targetVelocity=wl, force=max_motor_torque)
            p.setJointMotorControl2(robot_id, right_joint_index, p.VELOCITY_CONTROL, targetVelocity=wr, force=max_motor_torque)
            p.stepSimulation()
            if not fast_mode:
                #time.sleep(1.0/240.0)
                time.sleep(0.0003)


    

    finally:
        p.disconnect()

        if fitness_steps > 0:
            avg_fitness = fitness_sum / fitness_steps
            result_text = (
                f"Fitness promedio: {avg_fitness:.4f}\n"
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
        else:
            result_text = "No steps evaluated for fitness."

        # Mostrar popup solo si estamos en GUI principal
        if len(sys.argv) == 1:
            messagebox.showinfo("Resultados", result_text)

        # Salida segura según modo
        if use_gui:
            os._exit(0)  # Evita crash en macOS con PyBullet GUI
        else:
            sys.exit(0)  # Permite mostrar popup en modo DIRECT



# --- Tkinter GUI ---
if len(sys.argv) > 1:
    controller_file = sys.argv[1]
    steps = int(sys.argv[2])
    use_gui = sys.argv[3].lower() == "true"
    run_simulation(controller_file, steps, use_gui)
    sys.exit(0)

root = tk.Tk()
root.title("Simulación ePuck")

file_label = tk.Label(root, text="Archivo JSON:")
file_label.grid(row=0, column=0, padx=5, pady=5)
file_entry = tk.Entry(root, width=40)
file_entry.grid(row=0, column=1, padx=5, pady=5)

def browse_file():
    filename = filedialog.askopenfilename(title="Selecciona archivo JSON", filetypes=[("JSON files", "*.json")])
    file_entry.delete(0, tk.END)
    file_entry.insert(0, filename)

browse_btn = tk.Button(root, text="Buscar", command=browse_file)
browse_btn.grid(row=0, column=2, padx=5, pady=5)

steps_label = tk.Label(root, text="Número de pasos:")
steps_label.grid(row=1, column=0, padx=5, pady=5)
steps_entry = tk.Entry(root, width=10)
steps_entry.insert(0, "5000")
steps_entry.grid(row=1, column=1, padx=5, pady=5)

mode_var = tk.BooleanVar(value=True)
mode_check = tk.Checkbutton(root, text="Usar GUI (desactiva para DIRECT)", variable=mode_var)
mode_check.grid(row=2, column=0, columnspan=2, pady=5)

'''
def run_clicked():
    controller_file = file_entry.get()
    steps = int(steps_entry.get())
    use_gui = mode_var.get()
    if not controller_file:
        messagebox.showerror("Error", "Selecciona un archivo JSON")
        return
    run_simulation(controller_file, steps, use_gui)

run_btn = tk.Button(root, text="Ejecutar Simulación", command=run_clicked)
run_btn.grid(row=3, column=0, columnspan=3, pady=10)

root.mainloop()

def run_clicked():
    controller_file = file_entry.get()
    steps = steps_entry.get()
    use_gui = mode_var.get()
    if not controller_file:
        messagebox.showerror("Error", "Selecciona un archivo JSON")
        return

    # Launch simulation in a separate process
    args = ["python", "test_sim_epuck.py", controller_file, steps, str(use_gui)]
    subprocess.Popen(args)

run_btn = tk.Button(root, text="Ejecutar Simulación", command=run_clicked)
run_btn.grid(row=3, column=0, columnspan=3, pady=10)

root.mainloop()
'''


def run_clicked():
    controller_file = file_entry.get()
    steps = steps_entry.get()
    use_gui = mode_var.get()
    if not controller_file:
        messagebox.showerror("Error", "Selecciona un archivo JSON")
        return

    # Deshabilitar el botón mientras corre la simulación
    run_btn.config(state=tk.DISABLED)

    # Lanzar la simulación en un proceso separado
    args = ["python", "run_epuck_sim.py", controller_file, steps, str(use_gui)]
    proc = subprocess.Popen(args)

    '''
    # Función para verificar si el proceso terminó
    def check_process():
        if proc.poll() is None:
            root.after(1000, check_process)  # Revisa cada segundo
        else:
            run_btn.config(state=tk.NORMAL)  # Reactiva el botón cuando termina
    '''
    

    # Función para verificar si el proceso terminó
    def check_process():
        if proc.poll() is None:
            root.after(1000, check_process)  # Revisa cada segundo
        else:
            run_btn.config(state=tk.NORMAL)  # Reactiva el botón cuando termina
            #status_label.config(text="Simulación finalizada")

            # Leer resultados del JSON y mostrar popup
            try:
                with open("fitness_results.json", "r", encoding="utf-8") as f:
                    data = json.load(f)
                messagebox.showinfo("Resultados",
                    f"Fitness promedio: {data['avg_fitness']:.4f}\n"
                    f"Mean V: {data['mean_V']:.3f}\n"
                    f"Mean D: {data['mean_D']:.3f}\n"
                    f"Mean I: {data['mean_I']:.3f}"
                )
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo leer resultados: {e}")


    check_process()

run_btn = tk.Button(root, text="Ejecutar Simulación", command=run_clicked)
run_btn.grid(row=3, column=0, columnspan=3, pady=10)



status_label = tk.Label(root, text="", fg="blue")
status_label.grid(row=4, column=0, columnspan=3, pady=5)


root.mainloop()

