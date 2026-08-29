import math
from swarm_controller.grid_world_transform import GridWorldTransform

class VictimLocalizer:
    def __init__(self, fov_horizontal_deg=60.0, fov_vertical_deg=45.0):
        # Default FOV based on typical Gazebo camera specs
        self.fov_h = math.radians(fov_horizontal_deg)
        self.fov_v = math.radians(fov_vertical_deg)

    def localize(self, drone_world_x, drone_world_y, drone_world_z, drone_yaw, img_w, img_h, bbox):
        """
        Estimates the world coordinate of a victim given a bounding box and drone state.
        Assumptions for Phase N8:
        - The ground is flat (Z=0).
        - The camera is pointing straight down (pitch=-90 degrees) or slightly angled.
        - The camera is mounted exactly at the drone's origin (simplified).
        """
        x1, y1, x2, y2 = bbox
        
        # Center of bounding box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        
        # Normalized image coordinates [-1, 1]
        nx = (cx - (img_w / 2.0)) / (img_w / 2.0)
        ny = (cy - (img_h / 2.0)) / (img_h / 2.0)
        
        # We assume the camera looks straight down.
        # drone_world_z is the absolute altitude (e.g., 15m)
        alt = drone_world_z
        
        # Calculate offsets on the ground plane in the camera's local frame
        # nx = 1 means it's at the edge of horizontal FOV
        offset_y_local = nx * alt * math.tan(self.fov_h / 2.0)
        offset_x_local = -ny * alt * math.tan(self.fov_v / 2.0)
        
        # Rotate local offsets by drone yaw to get global offsets
        cos_yaw = math.cos(drone_yaw)
        sin_yaw = math.sin(drone_yaw)
        
        # Assuming camera x is forward, y is right
        offset_x_global = offset_x_local * cos_yaw - offset_y_local * sin_yaw
        offset_y_global = offset_x_local * sin_yaw + offset_y_local * cos_yaw
        
        victim_x = drone_world_x + offset_x_global
        victim_y = drone_world_y + offset_y_global
        
        # Convert to grid coordinates
        grid_x, grid_y = GridWorldTransform.world_to_grid(victim_x, victim_y)
        
        return {
            'world_x': victim_x,
            'world_y': victim_y,
            'grid_x': grid_x,
            'grid_y': grid_y
        }
