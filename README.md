# Evolutionary e-puck Simulator (PyBullet + Genetic Algorithm)

This project contains a PyBullet-based simulator of an e-puck-like differential drive robot and a genetic algorithm (GA) to evolve simple reactive controllers.

Core components:

- **`run_epuck_evol.py`** – Evolutionary loop using a bitmask rule-table controller.
- **`run_epuck_sim.py`** – Tkinter GUI simulator that loads controllers from JSON and computes Nolfi-style fitness.
- **`view_epuck_sim.py`** – Command-line visualizer for a given controller JSON.
- **`robots/`** – URDF model of the robot (`epuck_sim.urdf`).
- **`braitenberg/`** – Example Braitenberg-style controller(s).
- **`best_chromosomes/`** – Best individual(s) saved by the GA across generations.

---

## 1. Requirements

Python 3.8+ is recommended.

Install dependencies:

```bash
pip install -r requirements.txt
```

Typical content of `requirements.txt`:

```text
pybullet
numpy
matplotlib
# Tkinter is required for the GUI (run_epuck_sim.py),
# but is typically installed with Python, not via pip.
```

On many systems, Tkinter is installed with Python by default. If it’s missing, install it via your OS package manager.

---

## 2. Robot and world

### URDF model

In `robots/`:

- `epuck_sim.urdf` – small cylindrical differential-drive robot with two wheels (`left_wheel_joint`, `right_wheel_joint`) and a rear caster.

Scripts use:

- `run_epuck_evol.py` – loads `robots/epuck_sim.urdf`.
- `run_epuck_sim.py` and `view_epuck_sim.py` – should also be configured to use `epuck_sim.urdf`.

### World / arena

- Square arena of size `WORLD_SIZE = 1.5 m`.
- Four walls forming a box.
- Three static box obstacles.
- One blue cylindrical “recharge area” (visual only; no energy logic yet).

The GA and the simulators use matched versions of:

- `add_world()` – same walls, obstacles, recharge area.
- `read_range_sensors()` – 8 range sensors, ignoring self-hits.
- `random_start()` – robot spawns with a margin from the walls to avoid sensors starting saturated.

---

## 3. Controllers

Two encodings are supported by the simulators:

### 3.1 Bitmask rule-table controller

- Encoding: `"encoding": "bitmask"`.
- State: 3-bit mask `(left, center, right)` derived from 8 range sensors and a distance threshold.
- 8 rules (0–7). Each rule selects an action from a discrete set.

Example (`best_chromosomes/gen_0099.json`):

```json
{
  "generation": 99,
  "chromosome": [
    "wander",
    "forward",
    "wander",
    "wander",
    "wander",
    "left",
    "wander",
    "right"
  ],
  "fitness": 246.80868122674224,
  "encoding": "bitmask"
}
```

Actions are mapped to wheel speeds, e.g.:

- `forward`
- `left`
- `right`
- `reverse`
- `stop` (sometimes avoided to reduce staying still)
- `wander` (forward with a slight turn)

### 3.2 Braitenberg controller

- Encoding: `"encoding": "braitenberg"`.
- 8 sensor readings are linearly combined into left/right wheel speeds via weights and biases.

Example (`braitenberg/braitenberg.json`):

```json
{
  "encoding": "braitenberg",
  "left_weights":  [-1.2, -0.7, 0.2, 0.6, 1.0, 0.6, 0.2, -0.7],
  "right_weights": [-0.7, 0.2, 0.6, 1.0, 1.2, 0.6, 0.2, -0.7],
  "left_bias":   0.07,
  "right_bias":  0.07,
  "max_speed":   0.35
}
```

Both encodings are supported in `run_epuck_sim.py` and `view_epuck_sim.py`.

---

## 4. Running the GA (evolution loop)

The main evolutionary loop is in `run_epuck_evol.py`.

It:

1. Creates a population of bitmask chromosomes (tables mapping 8 states to actions).
2. Evaluates each chromosome in PyBullet (no GUI) using a movement / coverage based heuristic fitness.
3. Applies elitism, mutation, and crossover.
4. Saves the best chromosome for each generation into `best_chromosomes/gen_XXXX.json`.
5. Plots and updates curves for best and average fitness per generation and saves them as `fitness_plot.png`.

Run:

```bash
python run_epuck_evol.py
```

Key parameters (inside the script):

- `POP_SIZE = 24`
- `GENERATIONS = 100`
- `MUTATION_RATE = 0.2`
- `CROSSOVER_RATE = 0.7`
- `ELITISM = 2`
- `EVAL_REPEATS = 3` (simulation repetitions per chromosome)

---

## 5. Running the simulator with GUI (Tkinter)

`run_epuck_sim.py` provides a small GUI for loading and running a controller.

### GUI mode

Start:

```bash
python run_epuck_sim.py
```

In the window:

1. Click **“Buscar”** to select a controller JSON file:
   - Bitmask: e.g. `best_chromosomes/gen_0099.json`.
   - Braitenberg: e.g. `braitenberg/braitenberg.json`.
2. Set the number of steps (e.g. `5000`).
3. Check or uncheck **Use GUI (uncheck for DIRECT)**:
   - Checked: PyBullet opens a 3D GUI window.
   - Unchecked: runs in `DIRECT` mode (no GUI).
4. Click **“Ejecutar Simulación”**.

During GUI simulation:

- Press **space bar** in the PyBullet window to toggle fast mode / real-time.

After the simulation:

- `run_epuck_sim.py` writes `fitness_results.json` with:
  - `avg_fitness` (Nolfi-style),
  - `mean_V`, `mean_D`, `mean_I`.
- A popup shows these metrics if running under the main Tkinter GUI.

---

## 6. Running the visualizer from the command line

`view_epuck_sim.py` is a standalone visualizer:

```bash
python view_epuck_sim.py path/to/controller.json 5000
```

- `controller.json`: controller file with `"encoding": "bitmask"` or `"braitenberg"`.
- `5000`: number of steps (optional; default is 2000).

It opens the PyBullet GUI, runs the robot, then prints:

- Average Nolfi-style fitness.
- Mean V, D, and I over the episode.

---

## 7. GitHub workflow

Basic workflow once the repo is cloned:

```bash
# make changes
git status
git add <files>
git commit -m "Describe your change"
git push
```

Repository URL (for cloning):

```bash
git clone https://github.com/ferdiex/epuck-evol-sim.git
```
