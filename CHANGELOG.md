# Changelog

## [1.5.0] - 2026-02-13

### Added
- Complete evolution system (run_epuck_evol.py)
- Fitness plotting tool (plot_fitness.py)
- GUI viewer (viewer_epuck_sim.py)
- Centralized configuration (epuck_config.json)
- Auto-timestamped logs
- Keyboard controls: R (reset), H/J (gripper)
- Auto-detect gripper presence

### Changed
- Renamed run_epuck_sim.py → viewer_epuck_sim.py
- Removed matplotlib from evolution (stability)
- Larger fonts in GUI

### Fixed
- macOS PyBullet crash on window close
- Log file overwriting issue

## [1.0.0] - 2026-02-12

### Added
- Initial release
- Braitenberg controllers
- Bitmask controllers
- Basic viewer
- Manual gripper control
