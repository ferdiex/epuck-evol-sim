"""
Configuration loader for ePuck simulation.
Loads and validates parameters from epuck_config.json.
"""

import json
import os
import sys

DEFAULT_CONFIG = "epuck_config.json"  # ← Changed from "config.json"

def load_config(config_path=None):
    """
    Load configuration from JSON file with validation.
    
    Args:
        config_path: Path to config file (default: epuck_config.json)
        
    Returns:
        Dictionary with configuration parameters
    """
    if config_path is None:
        config_path = DEFAULT_CONFIG
    
    if not os.path.exists(config_path):
        print(f"ERROR: Config file not found: {config_path}")
        print(f"Please create {config_path} or specify path with --config")
        sys.exit(1)
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON in config file: {e}")
        sys.exit(1)
    
    # Validate critical parameters
    _validate_config(cfg)
    
    return cfg


def _validate_config(cfg):
    """Validate configuration parameters are in valid ranges."""
    
    # Check required sections exist
    required_sections = ['world', 'robot', 'sensors', 'output']
    for section in required_sections:
        if section not in cfg:
            raise ValueError(f"Missing required config section: '{section}'")
    
    # Sensor range
    sensor_range = cfg['sensors']['range']
    if not (0.02 <= sensor_range <= 0.15):
        raise ValueError(f"Sensor range {sensor_range} out of bounds [0.02, 0.15]")
    
    # World size
    world_size = cfg['world']['size']
    if not (0.5 <= world_size <= 5.0):
        raise ValueError(f"World size {world_size} out of bounds [0.5, 5.0]")
    
    # Verbosity
    verbosity = cfg['output']['verbosity']
    if verbosity not in [0, 1, 2, 3, 4]:
        raise ValueError(f"Verbosity {verbosity} must be 0-4")
    
    # Robot URDF exists
    urdf_path = cfg['robot']['urdf_path']
    if not os.path.exists(urdf_path):
        print(f"WARNING: Robot URDF not found: {urdf_path}")
    
    # Evolution-specific validation
    if 'evolution' in cfg:
        mut_rate = cfg['evolution']['mutation_rate']
        if not (0.0 <= mut_rate <= 1.0):
            raise ValueError(f"Mutation rate {mut_rate} must be in [0, 1]")
        
        cross_rate = cfg['evolution']['crossover_rate']
        if not (0.0 <= cross_rate <= 1.0):
            raise ValueError(f"Crossover rate {cross_rate} must be in [0, 1]")
    
    print(f"[CONFIG] Validated: sensor_range={sensor_range}m, "
          f"world_size={world_size}m, verbosity={verbosity}")


def get_param(cfg, *keys, default=None):
    """
    Safely get nested config parameter with fallback.
    
    Example:
        get_param(cfg, 'sensors', 'range', default=0.055)
    
    Args:
        cfg: Configuration dictionary
        *keys: Nested keys to traverse
        default: Fallback value if key not found
        
    Returns:
        Parameter value or default
    """
    result = cfg
    for key in keys:
        if isinstance(result, dict) and key in result:
            result = result[key]
        else:
            return default
    return result


def save_config_with_controller(controller_path, config):
    """
    Save config alongside controller JSON for reproducibility.
    
    Example:
        controller_gen_042.json -> controller_gen_042_config.json
    
    Args:
        controller_path: Path to controller JSON file
        config: Configuration dictionary to save
    """
    base = os.path.splitext(controller_path)[0]
    config_path = f"{base}_config.json"
    
    with open(config_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2)
    
    if config.get('output', {}).get('verbosity', 1) >= 1:
        print(f"[CONFIG] Saved config to: {config_path}")


def parse_config_arg(args=None):
    """
    Parse --config argument from command line.
    
    Args:
        args: List of arguments (default: sys.argv)
        
    Returns:
        Path to config file or None for default
    """
    if args is None:
        args = sys.argv
    
    if "--config" in args:
        idx = args.index("--config")
        if idx + 1 < len(args):
            return args[idx + 1]
        else:
            print("ERROR: --config flag requires a path argument")
            sys.exit(1)
    
    return None
    