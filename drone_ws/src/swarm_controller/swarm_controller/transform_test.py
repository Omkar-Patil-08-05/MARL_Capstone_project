import math

def local_to_world(local_x, local_y, world_spawn_x, world_spawn_y, world_yaw):
    # PX4 local is NED. X is forward, Y is right.
    # Gazebo world is ENU. X is forward (East), Y is left (North).
    # Assuming world_yaw is 0:
    # local_x (Forward) -> world_x (Forward)
    # local_y (Right) -> world_y (-Left = -world_y)
    
    # Let's use a general 2D rotation for yaw.
    # If world_yaw = 0, drone points East (+X).
    # local_x is East, local_y is South (-Y in ENU).
    # So world_dx = local_x * cos(yaw) - local_y * sin(yaw) ?
    # Wait, if yaw=0: world_dx = local_x, world_dy = -local_y.
    
    # Standard 2D rotation for ENU (where yaw=0 is +X, yaw=90 is +Y):
    # body_x in world = (cos(yaw), sin(yaw))
    # body_y (Right) in world = (sin(yaw), -cos(yaw))
    
    world_dx = local_x * math.cos(world_yaw) + local_y * math.sin(world_yaw)
    world_dy = local_x * math.sin(world_yaw) - local_y * math.cos(world_yaw)
    
    return world_spawn_x + world_dx, world_spawn_y + world_dy

def world_to_local(world_x, world_y, world_spawn_x, world_spawn_y, world_yaw):
    world_dx = world_x - world_spawn_x
    world_dy = world_y - world_spawn_y
    
    # Inverse of the above matrix:
    local_x = world_dx * math.cos(world_yaw) + world_dy * math.sin(world_yaw)
    local_y = world_dx * math.sin(world_yaw) - world_dy * math.cos(world_yaw)
    
    return local_x, local_y

print("Testing transforms with Yaw=0:")
wx, wy = local_to_world(10, 5, 24, 120, 0)
print(f"Local (10, 5) -> World ({wx}, {wy})  [Expected: 34, 115]")
lx, ly = world_to_local(wx, wy, 24, 120, 0)
print(f"World ({wx}, {wy}) -> Local ({lx}, {ly}) [Expected: 10, 5]")

print("Testing transforms with Yaw=pi/2:")
wx, wy = local_to_world(10, 5, 24, 120, math.pi/2)
print(f"Local (10, 5) -> World ({wx}, {wy})  [Expected: 29, 130]")
lx, ly = world_to_local(wx, wy, 24, 120, math.pi/2)
print(f"World ({wx}, {wy}) -> Local ({lx}, {ly}) [Expected: 10, 5]")

