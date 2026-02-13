import pybullet as p
import pybullet_data
import numpy as np
import random
import json
import os
import sys
import signal
import time

# ============================================================================
# LOAD CONFIGURATION
# ============================================================================

from config_loader import load_config, parse_config_arg
from world_builder import build_world_from_config

# Parse --config flag (or use default epuck_config.json)
CONFIG_PATH = parse_config_arg()
cfg = load_config(CONFIG_PATH)

# ============================================================================
# RANDOM SEED
# ============================================================================

if cfg['evolution']['random_seed'] is not None:
    random.seed(cfg['evolution']['random_seed'])
    print(f"[INFO] Using fixed random seed: {cfg['evolution']['random_seed']}")
else:
    random.seed(os.urandom(16))
    print("[INFO] Using random seed from os.urandom")

# ============================================================================
# GENETIC ALGORITHM PARAMETERS (loaded from config)
# ============================================================================

POP_SIZE = cfg['evolution']['population_size']
GENERATIONS = cfg['evolution']['generations']
MUTATION_RATE = cfg['evolution']['mutation_rate']
CROSSOVER_RATE = cfg['evolution']['crossover_rate']
ELITISM = cfg['evolution']['elitism']
EVAL_REPEATS = cfg['evolution']['eval_repeats']
EVAL_STEPS = cfg['evolution']['eval_steps']
RULES = 8  # 2^3 states for (left, center, right) bitmask

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
# ACTIONS (loaded from config)
# ============================================================================

ACTIONS = cfg['bitmask']['actions']
ACTION_TABLE = {k: tuple(v) for k, v in cfg['bitmask']['action_table'].items()}
FREE_ACTIONS = cfg['bitmask']['free_space_actions']

# ============================================================================
# FITNESS PARAMETERS (loaded from config)
# ============================================================================

FITNESS_CFG = cfg['fitness']['coverage']

# ============================================================================
# GLOBAL STATE FOR SIGNAL HANDLER
# ============================================================================

current_generation = 0
current_best_chromosome = None
current_best_fitness = None
evolution_interrupted = False
log_file = None

# ============================================================================
# OUTPUT LOGGING CLASS
# ============================================================================

class Tee:
    """Write to multiple streams simultaneously (console + file)."""
    def __init__(self, *files):
        self.files = files
    
    def write(self, data):
        for f in self.files:
            f.write(data)
            f.flush()
    
    def flush(self):
        for f in self.files:
            f.flush()

# ============================================================================
# GRACEFUL CTRL+C HANDLER
# ============================================================================

def signal_handler(sig, frame):
    """
    Handle Ctrl+C gracefully - save current state and exit cleanly.
    """
    global evolution_interrupted, log_file
    
    print("\n\n" + "="*60)
    print("[INTERRUPT] Ctrl+C detected - shutting down gracefully...")
    print("="*60)
    
    evolution_interrupted = True
    
    # Save current best if available
    if current_best_chromosome is not None and current_best_fitness is not None:
        outdir = cfg['output']['output_dir']
        if not os.path.exists(outdir):
            os.makedirs(outdir)
        
        fname = os.path.join(outdir, f"gen_{current_generation:04d}_interrupted.json")
        with open(fname, "w") as f:
            data = {
                "generation": current_generation,
                "chromosome": current_best_chromosome,
                "fitness": current_best_fitness,
                "encoding": "bitmask",
                "interrupted": True
            }
            json.dump(data, f, indent=2)
        print(f"[INTERRUPT] Saved current best to: {fname}")
    
    # Close log file
    if log_file:
        log_file.close()
        print("[INTERRUPT] Closed log file")
    
    # Disconnect any PyBullet instances (should be closed, but just in case)
    try:
        if p.isConnected():
            p.disconnect()
        print("[INTERRUPT] Disconnected PyBullet")
    except:
        pass
    
    print("="*60)
    print(f"[INTERRUPT] Evolution stopped at generation {current_generation}")
    print(f"[INTERRUPT] Best fitness so far: {current_best_fitness:.3f}" if current_best_fitness else "[INTERRUPT] No fitness data yet")
    print(f"[INTERRUPT] Saved controllers: {cfg['output']['output_dir']}/gen_*.json")
    print("="*60)
    print("[INTERRUPT] Cleanup complete - exiting now\n")
    
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def set_friction(robot_id):
    """Match friction config with visualizer."""
    wheel_names = ["left_wheel_joint", "right_wheel_joint"]
    wheel_friction = cfg['robot']['friction']['wheel']
    chassis_friction = cfg['robot']['friction']['chassis']
    caster_friction = cfg['robot']['friction']['caster']

    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode('utf-8')
        if name in wheel_names:
            p.changeDynamics(robot_id, j, lateralFriction=wheel_friction)
        elif "caster" in name:
            p.changeDynamics(robot_id, j, lateralFriction=caster_friction)
    
    p.changeDynamics(robot_id, -1, lateralFriction=chassis_friction)


def get_bitmask_state(sensor_values):
    """
    Converts sensor readings into a 3-bit mask for left, center, right.
    Returns: integer in range [0,7].
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
    return ACTION_TABLE[action]

# ============================================================================
# GENETIC OPERATORS
# ============================================================================

def mutate(chromosome, actions=ACTIONS, mutation_rate=MUTATION_RATE):
    """
    Randomly mutate entries in the chromosome (rule table).
    For bitmask 000 (index 0), force only advancing/safe actions.
    """
    new_chrom = chromosome[:]
    for i in range(len(new_chrom)):
        if random.random() < mutation_rate:
            if i == 0:
                new_chrom[i] = random.choice(FREE_ACTIONS)
            else:
                new_chrom[i] = random.choice(actions)
    
    # Always make sure bitmask 000 is never idle
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
    For bitmask 000, force an advancing or exploratory movement.
    """
    population = []
    for _ in range(pop_size):
        chrom = [random.choice(ACTIONS) for _ in range(RULES)]
        chrom[0] = random.choice(FREE_ACTIONS)
        population.append(chrom)
    return population

# ============================================================================
# SENSOR SYSTEM
# ============================================================================

def read_range_sensors(robot_id, sensor_angles_world, sensor_range):
    """
    Simulate 8 range sensors around the robot.
    Ignores self-hits: sensors do not detect the robot's own body.
    """
    base_pos, base_ori = p.getBasePositionAndOrientation(robot_id)
    base_mat = p.getMatrixFromQuaternion(base_ori)
    fwd_vec = np.array([base_mat[0], base_mat[3], 0])
    left_vec = np.array([base_mat[1], base_mat[4], 0])
    readings = []
    
    for a in sensor_angles_world:
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


def random_start():
    """
    Choose a random initial robot pose within the arena, away from walls.
    """
    margin = cfg['robot']['spawn_margin']
    x = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    y = random.uniform(-WORLD_SIZE/2+margin, WORLD_SIZE/2-margin)
    yaw = random.uniform(-np.pi, np.pi)
    return [x,y,0.035], p.getQuaternionFromEuler([0, 0, yaw])

# ============================================================================
# CONTROLLER EVALUATION
# ============================================================================

def eval_controller(chromosome, eval_steps=None, repeats=None, gui=False):
    """
    Evaluates a controller in simulation with different random starting conditions.
    Returns the average fitness using coverage-based metric.
    """
    if eval_steps is None:
        eval_steps = EVAL_STEPS
    if repeats is None:
        repeats = EVAL_REPEATS
    
    fits = []
    for _ in range(repeats):
        physics = p.GUI if gui else p.DIRECT
        c = p.connect(physics)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, cfg['world']['gravity'])
        p.setTimeStep(cfg['world']['timestep'])
        p.setRealTimeSimulation(1 if cfg['world']['realtime_simulation'] else 0)
        p.loadURDF(cfg['world']['plane_urdf'])
        
        # Build world from config
        build_world_from_config(cfg)

        start_pos, start_ori = random_start()
        robot_id = p.loadURDF(cfg['robot']['urdf_path'], start_pos, start_ori, useFixedBase=False)
        
        set_friction(robot_id)
        
        left_joint = right_joint = None
        for j in range(p.getNumJoints(robot_id)):
            n = p.getJointInfo(robot_id, j)[1].decode('utf-8')
            if n == "left_wheel_joint":
                left_joint = j
            if n == "right_wheel_joint":
                right_joint = j
        
        wheel_rad = cfg['robot']['wheel_radius']
        max_motor_torque = cfg['robot']['max_motor_torque']
        max_speed = cfg['robot']['max_speed']

        prev_pos, _ = p.getBasePositionAndOrientation(robot_id)
        first_steps_pos = None
        score_distance = 0.0
        score_wheel = 0.0
        stuck_penalty = 0.0
        wander_bonus = 0.0

        action_history = []
        visited_cells = set()

        for t in range(eval_steps):
            robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
            yaw = p.getEulerFromQuaternion(robot_ori)[2]
            global_angles = yaw + SENSOR_ANGLES
            sensor_vals = read_range_sensors(robot_id, global_angles, SENSOR_RANGE)

            if t == 5:
                first_steps_pos = robot_pos

            l_speed, r_speed = bitmask_table_controller(sensor_vals, chromosome)
            l_speed = np.clip(l_speed, -max_speed, max_speed)
            r_speed = np.clip(r_speed, -max_speed, max_speed)

            # Encourage moving when not sensing anything
            if get_bitmask_state(sensor_vals) == 0 and (l_speed == 0 and r_speed == 0):
                wander_bonus += FITNESS_CFG['wander_bonus_weight']

            # Penalize being entirely stopped
            if l_speed == 0 and r_speed == 0:
                stuck_penalty += FITNESS_CFG['stuck_penalty']

            p.setJointMotorControl2(robot_id, left_joint, p.VELOCITY_CONTROL, 
                                   targetVelocity=l_speed / wheel_rad, force=max_motor_torque)
            p.setJointMotorControl2(robot_id, right_joint, p.VELOCITY_CONTROL, 
                                   targetVelocity=r_speed / wheel_rad, force=max_motor_torque)
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
            prev_pos = curr_pos

        move_dist = np.linalg.norm(np.array(curr_pos)[:2] - np.array(start_pos)[:2])

        if move_dist < FITNESS_CFG['min_movement_threshold']:
            score = -1000.0  # Penalty for not leaving start area
        else:
            early_movement_bonus = 0.0
            if first_steps_pos:
                early_move = np.linalg.norm(np.array(first_steps_pos)[:2] - np.array(start_pos)[:2])
                if early_move > 0.04:
                    early_movement_bonus = FITNESS_CFG['early_movement_bonus']

            oscillation_penalty = 0.0
            if len(set(action_history)) < 3:
                oscillation_penalty = FITNESS_CFG['oscillation_penalty']

            unique_cells = len(visited_cells)
            coverage_bonus = FITNESS_CFG['coverage_weight'] * unique_cells

            score = (
                move_dist * FITNESS_CFG['distance_weight'] +
                FITNESS_CFG['wheel_weight'] * score_wheel +
                coverage_bonus +
                early_movement_bonus +
                wander_bonus -
                stuck_penalty -
                oscillation_penalty
            )

        fits.append(score)
        p.disconnect()
    
    return np.mean(fits)

# ============================================================================
# SAVING RESULTS
# ============================================================================

def save_chromosome(generation, chromosome, fitness, outdir=None):
    """
    Saves the best controller for a given generation to JSON.
    Config is saved separately (once) at start of evolution.
    """
    if outdir is None:
        outdir = cfg['output']['output_dir']
    
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

# ============================================================================
# MAIN EVOLUTION LOOP
# ============================================================================

def run_ga_and_save(pop_size=POP_SIZE, n_gen=GENERATIONS, eval_reps=EVAL_REPEATS, 
                    elitism=ELITISM, crossover_rate=CROSSOVER_RATE):
    """
    Main evolutionary loop: evolves controllers, includes crossover, 
    saves the best chromosome each generation.
    """
    global current_generation, current_best_chromosome, current_best_fitness, evolution_interrupted, log_file
    
    pop = make_population(pop_size)
    
    # ========================================
    # Setup output directory and logging
    # ========================================
    outdir = cfg['output']['output_dir']
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    
    # Save config ONCE at beginning
    config_save_path = os.path.join(outdir, "evolution_config.json")
    with open(config_save_path, 'w') as f:
        json.dump(cfg, f, indent=2)
    
    # Create timestamped log file
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_filename = f"evolution_{timestamp}.log"
    log_path = os.path.join(outdir, log_filename)
    log_file = open(log_path, 'w')
    
    # Redirect stdout to both console and log file
    original_stdout = sys.stdout
    sys.stdout = Tee(original_stdout, log_file)
    
    if VERBOSITY >= 1:
        print(f"[CONFIG] Saved evolution config to: {config_save_path}")
        print(f"[LOG] Logging to: {log_path}\n")
    # ========================================
    
    for gen in range(n_gen):
        # Check if interrupted
        if evolution_interrupted:
            break
        
        current_generation = gen
        
        fits = [eval_controller(chrom, repeats=eval_reps, gui=False) for chrom in pop]
        best_idx = np.argmax(fits)
        best_chromosome = pop[best_idx]
        best_fitness = fits[best_idx]
        avg_fitness = np.mean(fits)
        
        # Update global state for signal handler
        current_best_chromosome = best_chromosome
        current_best_fitness = best_fitness
        
        # Save every N generations (configurable)
        if gen % cfg['output']['save_best_every'] == 0:
            save_chromosome(gen, best_chromosome, best_fitness)
        
        if VERBOSITY >= 1:
            print(f"Gen {gen:4d} | best: {best_fitness:.3f} | avg: {avg_fitness:.3f} | rule: {best_chromosome}")
        
        # Next generation (elitism, crossover, mutation)
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
    
    # Restore original stdout and close log file
    sys.stdout = original_stdout
    log_file.close()

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    if VERBOSITY >= 1:
        print("\n" + "="*60)
        print("=== ePuck Evolution Starting ===")
        print("="*60)
        print(f"Config: {CONFIG_PATH if CONFIG_PATH else 'epuck_config.json (default)'}")
        print(f"Population: {POP_SIZE}")
        print(f"Generations: {GENERATIONS}")
        print(f"Sensor range: {SENSOR_RANGE}m")
        print(f"World type: {cfg['world']['type']}")
        print(f"Eval steps: {EVAL_STEPS}")
        print(f"Eval repeats: {EVAL_REPEATS}")
        print("="*60)
        print("\n💡 TIP: Plot results after evolution:")
        print("   python plot_fitness.py best_chromosomes/evolution_YYYYMMDD_HHMMSS.log")
        print("\n🛑 TIP: Press Ctrl+C to stop gracefully and save current progress")
        print("="*60 + "\n")

    try:
        run_ga_and_save()
    except KeyboardInterrupt:
        # Should be caught by signal handler, but just in case
        print("\n[FALLBACK] KeyboardInterrupt caught - exiting")
        pass
    
    # Normal completion (not interrupted)
    if not evolution_interrupted and VERBOSITY >= 1:
        print("\n" + "="*60)
        print("=== Evolution Complete ===")
        print(f"Best controllers saved to: {cfg['output']['output_dir']}")
        print(f"Log file: {cfg['output']['output_dir']}/evolution_*.log")
        print("\n💡 Plot results:")
        print(f"   python plot_fitness.py {cfg['output']['output_dir']}/evolution_*.log")
        print("="*60 + "\n")