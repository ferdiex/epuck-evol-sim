"""
world_builder.py - World construction from configuration

Builds PyBullet simulation environments based on epuck_config.json.

Supports:
- Basic walls (square arena) - Adjusted for e-puck gripper height
- Cylindrical obstacles (graspable size)
- Future: Mazes, targets, object spawning
"""

import pybullet as p
import random


def build_world_from_config(cfg):
    """
    Build complete world (walls + obstacles) from configuration.
    
    Args:
        cfg: Configuration dictionary loaded from epuck_config.json
    """
    world_type = cfg['world']['type']
    world_size = cfg['world']['size']
    
    # Build arena walls
    if world_type == "basic_walls":
        build_basic_walls(world_size)
    elif world_type == "obstacles":
        build_basic_walls(world_size)
    elif world_type == "maze":
        build_maze(world_size)
    else:
        print(f"[WARNING] Unknown world type: {world_type}, using basic_walls")
        build_basic_walls(world_size)
    
    # Build obstacles (if configured)
    if 'obstacles' in cfg['world']:
        if 'cylinders' in cfg['world']['obstacles']:
            cyl_cfg = cfg['world']['obstacles']['cylinders']
            if cyl_cfg.get('enabled', False):
                spawn_cylinders(
                    count=cyl_cfg['count'],
                    radius=cyl_cfg['radius'],
                    height=cyl_cfg['height'],
                    color=cyl_cfg['color'],
                    mass=cyl_cfg.get('mass', 0.0),
                    spawn_margin=cyl_cfg['spawn_margin'],
                    min_separation=cyl_cfg['min_separation'],
                    world_size=world_size,
                    random_seed=cyl_cfg.get('random_seed')
                )


def build_basic_walls(world_size):
    """
    Build square arena with 4 walls.
    Height adjusted for e-puck gripper (robot height ~5cm).
    
    Args:
        world_size: Side length of square arena (meters)
    """
    wall_thickness = 0.02
    wall_height = 0.08  # 8cm walls (was 30cm - too tall!)
    half_size = world_size / 2.0
    
    # Wall color (gray)
    wall_color = [0.7, 0.7, 0.7, 1.0]
    
    # Wall positions: [x, y, z]
    walls = [
        # North wall
        ([0, half_size, wall_height/2], 
         [world_size, wall_thickness, wall_height]),
        # South wall
        ([0, -half_size, wall_height/2], 
         [world_size, wall_thickness, wall_height]),
        # East wall
        ([half_size, 0, wall_height/2], 
         [wall_thickness, world_size, wall_height]),
        # West wall
        ([-half_size, 0, wall_height/2], 
         [wall_thickness, world_size, wall_height])
    ]
    
    for position, dimensions in walls:
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[d/2 for d in dimensions]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[d/2 for d in dimensions],
            rgbaColor=wall_color
        )
        
        wall_id = p.createMultiBody(
            baseMass=0,  # Static (immovable)
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        
        # Set high friction for walls
        p.changeDynamics(wall_id, -1, lateralFriction=1.0)
    
    print(f"[INFO] Built basic walls arena ({world_size}m x {world_size}m, walls={wall_height*100:.0f}cm)")


def spawn_cylinders(count, radius, height, color, mass, spawn_margin, 
                   min_separation, world_size, random_seed=None):
    """
    Spawn cylindrical obstacles randomly in the arena.
    Size adjusted to be graspable by e-puck gripper.
    
    Args:
        count: Number of cylinders to spawn
        radius: Cylinder radius (meters) - Recommended: 0.010-0.020 for graspable
        height: Cylinder height (meters) - Recommended: 0.03-0.05 for visibility
        color: RGBA color [r, g, b, a]
        mass: Cylinder mass (0 = static/fixed, >0 = dynamic/pushable)
        spawn_margin: Minimum distance from walls (meters)
        min_separation: Minimum distance between cylinders (meters)
        world_size: Arena size (meters)
        random_seed: Random seed for reproducibility (None = random)
    """
    if random_seed is not None:
        random.seed(random_seed)
    
    positions = []
    max_attempts = 100
    
    # Generate valid positions
    for i in range(count):
        placed = False
        for attempt in range(max_attempts):
            # Random position within bounds
            x = random.uniform(-world_size/2 + spawn_margin, 
                              world_size/2 - spawn_margin)
            y = random.uniform(-world_size/2 + spawn_margin, 
                              world_size/2 - spawn_margin)
            
            # Check separation from other cylinders
            valid = True
            for px, py in positions:
                dist = ((x - px)**2 + (y - py)**2)**0.5
                if dist < min_separation:
                    valid = False
                    break
            
            if valid:
                positions.append((x, y))
                placed = True
                break
        
        if not placed:
            print(f"[WARNING] Could only place {i} of {count} cylinders (not enough space)")
            break
    
    # Create cylinders in PyBullet
    for i, (x, y) in enumerate(positions):
        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=radius,
            height=height
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=radius,
            length=height,
            rgbaColor=color
        )
        
        cylinder_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[x, y, height/2]
        )
        
        # Set friction
        p.changeDynamics(cylinder_id, -1, lateralFriction=0.8)
    
    obstacle_type = "static" if mass == 0 else f"dynamic (mass={mass*1000:.0f}g)"
    print(f"[INFO] Spawned {len(positions)} {obstacle_type} cylinders (r={radius*100:.1f}cm, h={height*100:.1f}cm)")


def build_maze(world_size):
    """
    Build maze environment (future implementation).
    
    Args:
        world_size: Maze size (meters)
    """
    print("[WARNING] Maze world type not yet implemented, using basic_walls")
    build_basic_walls(world_size)
