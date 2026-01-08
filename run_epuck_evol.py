import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import random
import json
import os

# === Random Seed ===
# random.seed(42)  # Semilla fija para reproducibilidad
random.seed(os.urandom(16))

# === Genetic Algorithm parameters ===
POP_SIZE = 24
GENERATIONS = 100
MUTATION_RATE = 0.2
CROSSOVER_RATE = 0.7
ELITISM = 2
EVAL_REPEATS = 3
RULES = 8  # 2^3 states for (left, center, right) bitmask

# === States and Actions ===

# Bitmask state: (left_obstacle, front_obstacle, right_obstacle) -> 3 bits, gives 8 rules
# (0b000 = nothing close, 0b001 = obstacle right... up to 0b111 = obstacles all around)

ACTIONS = [
    "forward",      # move straight
    "left",         # turn left (left wheel slow, right wheel fast)
    "right",        # turn right (right wheel slow, left wheel fast)
    "reverse",      # both wheels backward
    "wander"        # move forward with a slight turn (for exploration)
    # "stop" is not included to avoid standing still by default
]

# Forbid spin-in-place and explicit stop as default moves to always keep moving
ACTION_TABLE = {
    "forward": (1.0, 1.0),
    "left":    (0.2, 1.0),
    "right":   (1.0, 0.2),
    "reverse": (-0.5, -0.5),
    "wander":  (0.6, 0.35),  # explore, roam and don't get stuck
}

# A reduced action set to use only for the bitmask 000 (no obstacles) state
FREE_ACTIONS = ["forward", "left", "right", "wander", "reverse"]

def set_friction(robot_id):
    # Match friction config with visualizer.
    wheel_names = ["left_wheel_joint", "right_wheel_joint"]
    wheel_friction = 1.5
    chassis_friction = 0.01
    caster_friction = 0.05

    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode('utf-8')
        if name in wheel_names:
            p.changeDynamics(robot_id, j, lateralFriction=wheel_friction)
        elif "caster" in name:
            p.changeDynamics(robot_id, j, lateralFriction=caster_friction)
    # Chassis (base link, usually -1)
    p.changeDynamics(robot_id, -1, lateralFriction=chassis_friction)

# === Sensor configuration ===
NUM_SENSORS = 8
SENSOR_RANGE = 0.10
SENSOR_ANGLES = np.linspace(-np.pi, np.pi, NUM_SENSORS, endpoint=False)
WORLD_SIZE = 1.5
OBSTACLE_THRESHOLD = 0.25

def get_bitmask_state(sensor_values):
    """
    Converts sensor readings into a 3-bit mask for left, center, right.
    Returns: integer in range [0,7].
    Each bit is 1 if ANY sensor in its region exceeds the obstacle threshold ("free" in that zone).
    """
    left = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [0,1])
    center = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [2,3,4,5])
    right = any(sensor_values[i] > OBSTACLE_THRESHOLD for i in [6,7])
    return (left << 2) | (center << 1) | (right << 0)

def bitmask_table_controller(sensor_values, chromosome):
    """
    Looks up the action for the current 3-bit mask state based on the genome.
    """
    idx = get_bitmask_state(sensor_values)
    action = chromosome[idx]
    # DEBUG PRINT:
    # print(f"[DEBUG] Bitmask: {idx:03b} (dec {idx}) | Action: {action} | Sensors: {np.round(sensor_values,2)}")
    return ACTION_TABLE[action]

# === Genetic Operators ===

def mutate(chromosome, actions=ACTIONS, mutation_rate=MUTATION_RATE):
    """
    Randomly mutate entries in the chromosome (rule table).
    For bitmask 000 (index 0), force only advancing/safe actions (never 'stop').
    """
    new_chrom = chromosome[:]
    for i in range(len(new_chrom)):
        if random.random() < mutation_rate:
            # For bitmask 000 (completely free), do not allow "stop"
            if i == 0:
                new_chrom[i] = random.choice(FREE_ACTIONS)
            else:
                new_chrom[i] = random.choice(actions)
    # Always make sure bitmask 000 is never "stop", prefer "forward":
    if new_chrom[0] not in FREE_ACTIONS:
        new_chrom[0] = "forward"
    return new_chrom

def crossover(parent1, parent2):
    """
    Uniform crossover: create a child by picking each gene randomly from parents.
    """
    return [random.choice([g1, g2]) for g1, g2 in zip(parent1, parent2)]

def make_population(pop_size):
    """
    Create a random population of rule tables.
    For bitmask 000, force an advancing or exploratory movement (never 'stop' or idle).
    """
    population = []
    for _ in range(pop_size):
        chrom = [random.choice(ACTIONS) for _ in range(RULES)]
        chrom[0] = random.choice(FREE_ACTIONS)
        population.append(chrom)
    return population

# === PyBullet World and Sensors ===

def add_world():
    """
    Sets up the arena with box walls, obstacles, and a recharge area
    (visual only for now, ready for future features).
    """
    half = WORLD_SIZE / 2.0
    height = 0.06
    thick = 0.03

    # Create Y direction walls
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2], rgbaColor=[.8, .8, .8, 1])
    p.createMultiBody(0, wall_col, wall_vis, [0, half, height/2])
    p.createMultiBody(0, wall_col, wall_vis, [0, -half, height/2])

    # Create X direction walls
    wall_col_x = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2])
    wall_vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2], rgbaColor=[.8, .8, .8, 1])
    p.createMultiBody(0, wall_col_x, wall_vis_x, [half, 0, height/2])
    p.createMultiBody(0, wall_col_x, wall_vis_x, [-half, 0, height/2])

    # Add static obstacles (example boxes)
    ob_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.03, .03, .035])
    ob_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[.03, .03, .035], rgbaColor=[1.0, 0.15, 0.15, 1])
    for x, y in [(-0.30, 0.22), (0.38, 0.08), (0.0, -0.38)]:
        p.createMultiBody(0, ob_col, ob_vis, [x, y, 0.035])

    # Recharge area (blue cylinder, visual + collision, ready for future use)
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

def read_range_sensors(robot_id, sensor_angles_world, sensor_range, debug=False):
    """
    Simulate 8 range sensors around the robot. Each returns normalized distance (0=far, 1=close).
    Ignores self-hits: sensors do not detect the robot's own body.
    """
    base_pos, base_ori = p.getBasePositionAndOrientation(robot_id)
    base_mat = p.getMatrixFromQuaternion(base_ori)
    fwd_vec = np.array([base_mat[0], base_mat[3], 0])
    left_vec = np.array([base_mat[1], base_mat[4], 0])
    readings = []
    for a in sensor_angles_world:
        direction = np.cos(a) * fwd_vec + np.sin(a) * left_vec
        start = np.array(base_pos) + np.array([0, 0, 0.015])  # Start 1.5cm above ground
        end = start + direction * sensor_range
        ray = p.rayTest(start, end)[0]
        hit_fraction = ray[2]
        # Ignore self-hits (robot_id): treat as open space
        if ray[0] == -1 or ray[0] == robot_id:
            readings.append(0.0)
        else:
            readings.append(1.0 - hit_fraction)
        if debug:
            color = [1,0,0] if ray[0]==-1 or ray[0]==robot_id else [0,1,0]
            p.addUserDebugLine(start, end if ray[0]==-1 or ray[0]==robot_id else start + direction * (sensor_range * hit_fraction), color, 1.5, 0.05)
    return readings

def random_start():
    """
    Choose a random initial robot pose within the arena, away from walls.
    Margin increased for consistency with the visualizer, prevents immediate wall proximity.
    """
    margin = 0.4  # Larger margin (was 0.15) to avoid spawning next to walls
    x = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    y = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    yaw = random.uniform(-np.pi, np.pi)
    return [x,y,0.035], p.getQuaternionFromEuler([0, 0, yaw])

def random_start_flipped():
    """
    For diagnostic: Start upside-down, nose up, caster up.
    """
    margin = 0.4  # Match random_start margin
    x = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    y = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    yaw = random.uniform(-np.pi, np.pi)
    roll = np.pi
    pitch = 0
    return [x,y,0.035], p.getQuaternionFromEuler([roll, pitch, yaw])

def eval_controller(chromosome, eval_steps=2000, repeats=3, gui=False):
    """
    Evaluates a controller in simulation with different random starting conditions.
    Returns the average fitness.
    Fitness here is a coverage and movement-based heuristic, not Nolfi (can be swapped in later).
    """
    fits = []
    for _ in range(repeats):
        physics = p.GUI if gui else p.DIRECT
        c = p.connect(physics)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        add_world()

        # --- RANDOMIZE START every run ---
        start_pos, start_ori = random_start()
        # For flip-test: comment above, uncomment below
        # start_pos, start_ori = random_start_flipped()
        robot_id = p.loadURDF("robots/epuck_sim.urdf", start_pos, start_ori, useFixedBase=False)
        
        # Set friction for wheels/chassis/caster just like in visualizer!
        set_friction(robot_id)
        
        left_joint = right_joint = None
        for j in range(p.getNumJoints(robot_id)):
            n = p.getJointInfo(robot_id, j)[1].decode('utf-8')
            if n == "left_wheel_joint":
                left_joint = j
            if n == "right_wheel_joint":
                right_joint = j
        wheel_rad = 0.02
        max_motor_torque = 0.9
        max_speed = 0.35

        prev_pos, _ = p.getBasePositionAndOrientation(robot_id)
        first_steps_pos = None
        score_distance = 0.0
        score_wheel = 0.0
        stuck_penalty = 0.0
        wander_bonus = 0.0

        action_history = []
        path_history = [] 
        visited_cells = set()

        for t in range(eval_steps):
            robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
            yaw = p.getEulerFromQuaternion(robot_ori)[2]
            global_angles = yaw + SENSOR_ANGLES
            sensor_vals = read_range_sensors(robot_id, global_angles, SENSOR_RANGE, debug=False)

            if t == 5:
                first_steps_pos = robot_pos

            l_speed, r_speed = bitmask_table_controller(sensor_vals, chromosome)
            l_speed = np.clip(l_speed, -max_speed, max_speed)
            r_speed = np.clip(r_speed, -max_speed, max_speed)

            # Encourage moving when not sensing anything (avoid standing still)
            if get_bitmask_state(sensor_vals) == 0 and (l_speed == 0 and r_speed == 0):
                wander_bonus += 0.5  # larger bonus to help exploration

            # Penalize being entirely stopped in *any* state
            if l_speed == 0 and r_speed == 0:
                stuck_penalty += 0.2

            p.setJointMotorControl2(robot_id, left_joint, p.VELOCITY_CONTROL, targetVelocity=l_speed / wheel_rad, force=max_motor_torque)
            p.setJointMotorControl2(robot_id, right_joint, p.VELOCITY_CONTROL, targetVelocity=r_speed / wheel_rad, force=max_motor_torque)
            p.stepSimulation()
            curr_pos, _ = p.getBasePositionAndOrientation(robot_id)
            delta_y = curr_pos[1] - prev_pos[1]
            score_distance += delta_y
            wheel_speed_sum = abs(l_speed) + abs(r_speed)
            score_wheel += wheel_speed_sum

            cell = (int(curr_pos[0]*10), int(curr_pos[1]*10))
            visited_cells.add(cell)
            action = (round(l_speed, 2), round(r_speed, 2))
            action_history.append(action)
            if len(action_history) > 20:
                action_history.pop(0)
            path_history.append(curr_pos)
            prev_pos = curr_pos

        move_dist = np.linalg.norm(np.array(curr_pos)[:2] - np.array(start_pos)[:2])

        if move_dist < 0.12:
            score = -1000.0  # Penalty for not leaving start area
        else:
            early_movement_bonus = 0.0
            if first_steps_pos:
                early_move = np.linalg.norm(np.array(first_steps_pos)[:2] - np.array(start_pos)[:2])
                if early_move > 0.04:
                    early_movement_bonus = 1.0

            oscillation_penalty = 0.0
            if len(set(action_history)) < 3:
                oscillation_penalty = 2.0

            unique_cells = len(visited_cells)
            coverage_bonus = 0.05 * unique_cells

            score = (
                move_dist * 40 +             
                0.15 * score_wheel +         
                coverage_bonus +
                early_movement_bonus +
                wander_bonus -
                stuck_penalty -
                oscillation_penalty
            )

        fits.append(score)
        p.disconnect()
    return np.mean(fits)

def save_chromosome(generation, chromosome, fitness, outdir="best_chromosomes"):
    """
    Saves the best controller for a given generation to JSON.
    """
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    fname = os.path.join(outdir, f"gen_{generation:04d}.json")
    with open(fname, "w") as f:
        data = {
            "generation": generation,
            "chromosome": chromosome,
            "fitness": fitness,
            "encoding": "bitmask"
        }
        json.dump(data, f, indent=2)

def run_ga_and_save(pop_size=POP_SIZE, n_gen=GENERATIONS, eval_reps=EVAL_REPEATS, elitism=ELITISM, crossover_rate=CROSSOVER_RATE):
    """
    Main evolutionary loop: evolves controllers, includes crossover, saves the best chromosome each generation.
    """
    pop = make_population(pop_size)
    for gen in range(n_gen):
        fits = [eval_controller(chrom, repeats=eval_reps, gui=False) for chrom in pop]
        best_idx = np.argmax(fits)
        best_chromosome = pop[best_idx]
        best_fitness = fits[best_idx]
        avg_fitness = np.mean(fits)
        save_chromosome(gen, best_chromosome, best_fitness)
        
        # -- Update fitness curves for visualization --
        best_curve.append(best_fitness)
        avg_curve.append(avg_fitness)
        line1.set_data(range(len(best_curve)), best_curve)
        line2.set_data(range(len(avg_curve)), avg_curve)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.flush_events()
        plt.pause(0.001)
        
        print(f"Gen {gen:4d} | best: {best_fitness:.3f} | avg: {np.mean(fits):.3f} | rule: {best_chromosome}")
        # --- Next generation (elitism, crossover, mutation) ---
        next_pop = [pop[i] for i in np.argsort(fits)[-elitism:]]
        while len(next_pop) < pop_size:
            if random.random() < crossover_rate:
                parent1, parent2 = random.sample(pop, 2)
                child = crossover(parent1, parent2)
                child = mutate(child)
            else:
                parent = random.choice(pop)
                child = mutate(parent)
            next_pop.append(child)
        pop = next_pop

if __name__ == "__main__":
    # Set up plotting
    plt.ion()
    fig, ax = plt.subplots()
    best_curve = []
    avg_curve = []
    line1, = ax.plot([], [], label="Best Fitness")
    line2, = ax.plot([], [], label="Average Fitness")
    ax.set_xlabel("Generation")
    ax.set_ylabel("Fitness")
    ax.legend()

    run_ga_and_save()
    
    plt.ioff()
    plt.show()
    fig.savefig('fitness_plot.png')