import random
import os

# ==========================================================
# Forest World Bounds & Config
# ==========================================================
WORLD_SIZE = 500.0         # 500m x 500m physical terrain
HEIGHTMAP_RESOLUTION = 513 # 2^n + 1 (standard for Gazebo)
MAX_HEIGHT = 70.0          # max elevation in meters (ridge peaks)

WATER_LEVEL = 3.5          # water surface elevation (strictly inside riverbed)
RIVER_WIDTH = 18.0         # average width of river channel
RIVER_DEPTH = 5.0          # depth below terrain

# Operational Zones
BASE_SIZE = 65.0           # 65m diameter clearing for SAR drone base & staging
BASE_CLEARING_RADIUS = 32.0# Strict 0-tree exclusion radius around base center
SETTLEMENT_SIZE = 50.0

# Search & Rescue Operational Region
SEARCH_REGION_SIZE = 200.0 # 200m x 200m active SAR operational area

DEFAULT_TREE_COUNT = 950   # Dense woodland forest canopy (950 authentic 3D trees: 80 collision, 870 visual)
DEFAULT_UNDERSTORY_COUNT = 120 # Forest shrubs and understory bushes
DEFAULT_LOG_COUNT = 30     # Fallen logs and dead stumps
DEFAULT_ROCK_COUNT = 30    # Riverbank and ridge boulders
DEFAULT_CAST_SHADOWS = False # Disabled for mobile GPU RTF protection

# ==========================================================
# Victim Placement & Ground Contact Config
# ==========================================================
VICTIM_GROUND_OFFSET = 0.02        # meters above terrain surface for model origin
VICTIM_MAX_SLOPE_DEG = 8.0         # max terrain slope (deg); restricts to walkable clearings
VICTIM_MIN_TREE_CLEARANCE = 3.2    # meters; ensures clear ground around tree trunks
VICTIM_MIN_WATER_CLEARANCE_Z = 2.5 # meters above WATER_LEVEL
VICTIM_MIN_INTER_DISTANCE = 18.0   # meters between victims for spatial dispersion
VICTIM_BOUNDARY_MARGIN = 25.0      # meters from edge of 500m world

# ==========================================================
# Assets Root
# ==========================================================
ASSET_ROOT = os.path.expanduser("~/capstone_project_antigravity/assets_real")

# Shared deterministic RNG
RNG = random.Random(42)

def set_forest_seed(seed: int) -> None:
    """Reseed the shared RNG in-place so all modules pick up the new seed."""
    RNG.seed(seed)

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))
