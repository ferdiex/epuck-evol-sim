## Evolution System

### How It Works

The genetic algorithm evolves bitmask controllers - lookup tables mapping sensor states to actions:

1. Initialize random population of controllers
2. Evaluate each controller in simulation (fitness = navigation quality)
3. Select best controllers (elitism + fitness-proportional)
4. Reproduce via crossover and mutation
5. Repeat for N generations

Encoding: 8-rule lookup table for 3-bit sensor states (left/center/right obstacles)

Fitness Function: Coverage-based metric rewarding:
- Distance traveled
- Area explored (unique cells visited)
- Wheel activity (avoiding stuck states)
- Early movement (leaving start area quickly)

### Configuration

All parameters in epuck_config.json:

"evolution": {
  "population_size": 24,        // Number of individuals
  "generations": 100,           // Evolution iterations
  "mutation_rate": 0.15,        // Gene change probability
  "crossover_rate": 0.7,        // Parent mixing probability
  "elitism": 2,                 // Top N kept unchanged
  "eval_repeats": 3,            // Tests per controller
  "eval_steps": 2000,           // Steps per test
  "random_seed": null           // Set for reproducibility
}

### Output Files

Generated in best_chromosomes/ directory:

File                             Description
evolution_config.json            Snapshot of config used (reproducibility)
evolution_YYYYMMDD_HHMMSS.log    Fitness log with timestamp (never overwrites)
gen_0000.json                    Best controller from generation 0
gen_0050.json                    Best controller from generation 50
gen_0099.json                    Best controller from generation 99 (final)

Log format:
Gen    0 | best: 248.334 | avg: -180.908 | rule: ['reverse', 'reverse', ...]
Gen    1 | best: 260.579 | avg: -70.411 | rule: ['reverse', 'reverse', ...]
