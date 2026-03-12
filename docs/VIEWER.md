## Quick Command Reference

### Main Commands

| Task | Command |
|------|---------|
| **Run evolution** | `python run_epuck_evol.py` |
| **Run evolution (custom config)** | `python run_epuck_evol.py --config my_config.json` |
| **Plot evolution results** | `python plot_fitness.py best_chromosomes/evolution_*.log` |
| **Plot with custom title** | `python plot_fitness.py evolution.log --title "My Experiment"` |
| **Save plot to file** | `python plot_fitness.py evolution.log --save fitness.png` |
| **Test controller (CLI)** | `python view_epuck_sim.py controller.json 5000` |
| **Test controller (GUI)** | `python viewer_epuck_sim.py` |
| **Monitor evolution live** | `tail -f best_chromosomes/evolution_*.log` |

### Keyboard Controls (During Simulation)

| Key | Action |
|-----|--------|
| `SPACE` | Toggle fast/real-time mode |
| `W` | Toggle wireframe mode (Cmd+W on macOS, Alt+W on Windows) |
| `P` | Pause/resume simulation |
| `R` | Reset robot to initial position |
| `H` | Open gripper (if gripper detected) |
| `J` | Close gripper (if gripper detected) |
| `G` | Toggle fullscreen (PyBullet built-in) |
| `Ctrl+C` | Stop evolution gracefully (saves progress) |

---

### Testing Controllers

#### Command-Line (Advanced Users)

python view_epuck_sim.py best_chromosomes/gen_0099.json 5000

Arguments:
- controller.json - Controller file (required)
- 5000 - Number of simulation steps (optional, default: 10000)

Features:
- Verbose console output
- Keyboard controls (SPACE, P, R, H, J)
- Fitness results printed at end

---

#### GUI Launcher (Beginners)

python viewer_epuck_sim.py

Features:
- File browser to select controller
- Input field for number of steps
- GUI/DIRECT mode toggle
- Results popup at end
- Sa