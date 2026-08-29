import math
from typing import Tuple

class GridWorldTransform:
    """
    Handles coordinate mapping between the discrete SARGridEnv and continuous Gazebo world.
    Gazebo world coordinates are continuous meters [0, 100].
    SAR grid coordinates are row/col indices [0, 24].
    """
    METERS_PER_CELL = 4.0
    GRID_ORIGIN_X = 0.0
    GRID_ORIGIN_Y = 0.0
    GRID_SIZE_X = 25
    GRID_SIZE_Y = 25

    @classmethod
    def set_bounds(cls, width: int, height: int, meters_per_cell: float = 4.0, origin_x: float = 0.0, origin_y: float = 0.0):
        cls.GRID_SIZE_X = width
        cls.GRID_SIZE_Y = height
        cls.METERS_PER_CELL = meters_per_cell
        cls.GRID_ORIGIN_X = origin_x
        cls.GRID_ORIGIN_Y = origin_y

    @classmethod
    def grid_to_world_center(cls, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Returns the world coordinates for the center of the specified grid cell."""
        world_x = cls.GRID_ORIGIN_X + (grid_x + 0.5) * cls.METERS_PER_CELL
        world_y = cls.GRID_ORIGIN_Y + (grid_y + 0.5) * cls.METERS_PER_CELL
        return world_x, world_y

    @classmethod
    def grid_to_world_origin(cls, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Returns the world coordinates for the bottom-left origin of the grid cell.
        Note: The current PX4 mission spawn coordinates (12, 12) and (12, 20) correspond 
        to the cell origins of grid(3, 3) and grid(3, 5).
        """
        world_x = cls.GRID_ORIGIN_X + grid_x * cls.METERS_PER_CELL
        world_y = cls.GRID_ORIGIN_Y + grid_y * cls.METERS_PER_CELL
        return world_x, world_y

    @classmethod
    def world_to_grid(cls, world_x: float, world_y: float) -> Tuple[int, int]:
        """Converts world coordinates into grid cell indices (unclamped)."""
        grid_x = int(math.floor((world_x - cls.GRID_ORIGIN_X) / cls.METERS_PER_CELL))
        grid_y = int(math.floor((world_y - cls.GRID_ORIGIN_Y) / cls.METERS_PER_CELL))
        return grid_x, grid_y
        
    @classmethod
    def clamp_grid(cls, grid_x: int, grid_y: int) -> Tuple[int, int, bool]:
        """
        Clamps the requested grid target to the valid SAR environment boundaries.
        Returns:
            safe_grid_x, safe_grid_y, is_valid
            
        If is_valid is False, the requested target was out of bounds.
        """
        is_valid = True
        safe_x = grid_x
        safe_y = grid_y
        
        if safe_x < 0:
            safe_x = 0
            is_valid = False
        elif safe_x >= cls.GRID_SIZE_X:
            safe_x = cls.GRID_SIZE_X - 1
            is_valid = False
            
        if safe_y < 0:
            safe_y = 0
            is_valid = False
        elif safe_y >= cls.GRID_SIZE_Y:
            safe_y = cls.GRID_SIZE_Y - 1
            is_valid = False
            
        return safe_x, safe_y, is_valid
