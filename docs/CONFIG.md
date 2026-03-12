## Configuration System

All simulation parameters are centralized in epuck_config.json:

### Key Sections

Robot Physics:
"robot": {
  "urdf_path": "robots/epuck_sim.urdf",
  "wheel_radius": 0.0205,
  "max_speed": 0.35,
  "max_motor_torque": 0.5,
  "friction": {
    "wheel": 2.0,      // High traction
    "chassis": 0.3,    // Low drag
    "caster": 0.1      // Smooth rolling
  },
  "spawn_margin": 0.2  // Distance from walls at start
}

Sensor Configuration:
"sensors": {
  "num_sensors": 8,
  "range": 0.055,             // 5.5cm detection range
  "obstacle_threshold": 0.25  // 25% reading = obstacle detected
}

World Setup:
"world": {
  "type": "basic_walls",      // basic_walls, obstacles, maze
  "size": 1.5,                // Arena dimensions (1.5m x 1.5m)
  "gravity": -9.81,
  "timestep": 0.01,
  "plane_urdf": "plane.urdf"
}

Fitness Functions:
"fitness": {
  "nolfi": {
    "max_speed": 0.35
  },
  "coverage": {
    "distance_weight": 100.0,
    "wheel_weight": 0.1,
    "coverage_weight": 2.0,
    "min_movement_threshold": 0.05
  }
}

Evolution Parameters:
"evolution": {
  "population_size": 24,
  "generations": 100,
  "mutation_rate": 0.15,
  "crossover_rate": 0.7,
  "elitism": 2,
  "eval_repeats": 3,
  "eval_steps": 2000
}

### Using Custom Configs

# Create custom config
cp epuck_config.json my_experiment.json

# Edit parameters
nano my_experiment.json

# Run with custom config
python run_epuck_evol.py --config my_experiment.json
python view_epuck_sim.py controller.json 5000 --config my_experiment.json

---

## Simulation Parameters

### Configurable Constants

Sensor Configuration:
NUM_SENSORS = 8           # Number of proximity sensors
SENSOR_RANGE = 0.055      # Detection range (5.5cm)
OBSTACLE_THRESHOLD = 0.25 # Minimum reading to count as obstacle

Arena:
WORLD_SIZE = 1.5          # Arena dimensions (1.5m x 1.5m)

Debug Output:
VERBOSITY = 1             # 0=silent, 1=normal, 2=verbose
DEBUG_PRINT_FREQ = 50     # Console output frequency (frames)

---