import random

# ==========================================================
# Authoritative SAR World Bounds (FROZEN 100x100 grid)
# ==========================================================
WORLD_MIN_X = 0.0
WORLD_MAX_X = 100.0
WORLD_MIN_Y = 0.0
WORLD_MAX_Y = 100.0

# ==========================================================
# Compact Realistic City 
# Target: 75m x 75m footprint inside the 100x100m world
# ==========================================================
ROWS = 3
COLS = 3

SPACING = 25.0
ROAD_WIDTH = 3.0
BLOCK_SIZE = SPACING - ROAD_WIDTH
SIDEWALK = 1.0
TOWERS = 1

# Start coordinates (bottom left corner of the road grid)
CITY_WIDTH = COLS * SPACING
CITY_HEIGHT = ROWS * SPACING
START_X_ROAD = WORLD_MIN_X + (WORLD_MAX_X - WORLD_MIN_X - CITY_WIDTH) / 2.0
START_Y_ROAD = WORLD_MIN_Y + (WORLD_MAX_Y - WORLD_MIN_Y - CITY_HEIGHT) / 2.0

GROUND = {
    "concrete": ("0.3 0.3 0.3 1", "0.3 0.3 0.3 1"),
    "dirt": ("0.35 0.25 0.15 1", "0.35 0.25 0.15 1"),
    "grass": ("0.1 0.4 0.1 1", "0.1 0.4 0.1 1")
}
GROUND_TYPE = "concrete"

RNG = random.Random(42)  # Deterministic seed

def clamp_bounds(val, min_bound, max_bound):
    return max(min_bound, min(val, max_bound))