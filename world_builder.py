"""
World building utilities for ePuck simulation.
Defines different arena configurations.
"""

import pybullet as p


def build_world_from_config(cfg):
    """
    Build world based on config specification.
    
    Args:
        cfg: Configuration dictionary with 'world' section
    """
    world_cfg = cfg['world']
    world_type = world_cfg.get('type', 'basic_walls')
    world_size = world_cfg['size']
    
    if world_type == 'basic_walls':
        add_basic_walls(world_size)
    else:
        raise ValueError(f"Unknown world type: {world_type}")


def add_basic_walls(world_size):
    """
    Creates a square arena with walls and bounciness.
    
    Args:
        world_size: Arena size in meters (square)
    """
    half = world_size / 2.0
    height = 0.04
    thick = 0.03

    # Walls along Y-axis (north/south)
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, thick/2, height/2], 
                                    rgbaColor=[.8, .8, .8, 1])
    north_wall = p.createMultiBody(0, wall_col, wall_vis, [0, half, height/2])
    south_wall = p.createMultiBody(0, wall_col, wall_vis, [0, -half, height/2])

    # Walls along X-axis (east/west)
    wall_col_x = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2])
    wall_vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[thick/2, half, height/2], 
                                      rgbaColor=[.8, .8, .8, 1])
    east_wall = p.createMultiBody(0, wall_col_x, wall_vis_x, [half, 0, height/2])
    west_wall = p.createMultiBody(0, wall_col_x, wall_vis_x, [-half, 0, height/2])
    
    # Add slight bounciness to help break corner traps
    for wall_id in [north_wall, south_wall, east_wall, west_wall]:
        p.changeDynamics(wall_id, -1, restitution=0.2, lateralFriction=0.4)