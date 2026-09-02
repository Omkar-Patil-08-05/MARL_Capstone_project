import math
import random
import os
from config import *
from assets import AssetLibrary, ASSET_METADATA

assets = AssetLibrary()

# ==========================================================
# Metadata Tracking
# ==========================================================
WORLD_METADATA = {
    "obstacles": [],
    "victims": [],
    "drone_base": {}
}
OBJ_COUNTER = 0
VICTIM_INTENTS = []

# ==========================================================
# Configurable Victim Placement
# ==========================================================
_VICTIM_COUNT = 5
_VICTIM_SEED = 42

def set_victim_config(victim_count=5, seed=42):
    """Set victim count and seed before calling generate_city().
    Called from generate_world.py with CLI arguments."""
    global _VICTIM_COUNT, _VICTIM_SEED
    _VICTIM_COUNT = victim_count
    _VICTIM_SEED = seed

def add_obstacle_metadata(obj_id, category, asset, pose, aabb):
    type_str = "building"
    if category == "Rubble": type_str = "rubble"
    elif category == "Cars": type_str = "vehicle"
    elif category == "Towers": type_str = "tower"

    WORLD_METADATA["obstacles"].append({
        "id": obj_id,
        "type": type_str,
        "asset": asset,
        "pose": pose,
        "aabb": {
            "min_x": clamp_bounds(float(aabb[0]), WORLD_MIN_X, WORLD_MAX_X),
            "max_x": clamp_bounds(float(aabb[1]), WORLD_MIN_X, WORLD_MAX_X),
            "min_y": clamp_bounds(float(aabb[2]), WORLD_MIN_Y, WORLD_MAX_Y),
            "max_y": clamp_bounds(float(aabb[3]), WORLD_MIN_Y, WORLD_MAX_Y)
        }
    })

def add_victim_metadata(victim_id, wx, wy, wz=0.25):
    # Valid grid index requires clamping to valid traversable grid coordinates
    grid_x = int(math.floor(wx / 4.0))
    grid_y = int(math.floor(wy / 4.0))
    WORLD_METADATA["victims"].append({
        "id": victim_id,
        "world": {"x": float(wx), "y": float(wy), "z": float(wz)},
        "grid": {"x": grid_x, "y": grid_y}
    })

# ==========================================================
# City Geometry Center
# ==========================================================
CITY_WIDTH = COLS * SPACING
CITY_HEIGHT = ROWS * SPACING
# Centering the 75x75 city in the 100x100 grid gives a 12.5 margin on all sides.
START_X = WORLD_MIN_X + (WORLD_MAX_X - WORLD_MIN_X - CITY_WIDTH) / 2.0 + SPACING / 2.0
START_Y = WORLD_MIN_Y + (WORLD_MAX_Y - WORLD_MIN_Y - CITY_HEIGHT) / 2.0 + SPACING / 2.0

BUILDING_SETBACK = 1.0
BUILDING_CLEARANCE = 1.0

# Known Origin Fixes (to counter uncentered GLB origins)
XY_OFFSETS = {
    "abandoned_construction_building.glb": (-7.8786, 7.5000),
    "low_poly_-_soviet__apartment_building_8k.glb": (3.9908, -0.0070),
    "post-apocalyptic_buildings.glb": (0.0058, 21.3632),
    "futuristic_building.glb": (-2.6602, -4.1421),
    "residential_complex_modern_apartment_building.glb": (16.5009, 0.5013),
    "house.glb": (-4.8856, -2.6729),
    "small_house.glb": (-0.0032, -0.5276),
    "small_price_car.glb": (0.0000, 0.8379),
    "old_rusty_car.glb": (-0.0090, -0.1974),
    "abandoned_building.glb": (0.0, 0.0),
    "brick_rubble_scaniverse_lidar.glb": (0.0, 0.0)
}

# ==========================================================
# Layout Maps
# ==========================================================
LAYOUT_3x3 = [
    ["Drone Base", "Residential", "Commercial"],
    ["Park", "Major Damage", "Civic"],
    ["Industrial", "Mixed", "Collapsed"]
]

# ==========================================================
# Helper Functions
# ==========================================================
def spawn_specific(name, filename, category, x, y, yaw=0.0, z=0.0, collision_xml=""):
    original_list = assets.assets.get(category, [])
    for path in original_list:
        if os.path.basename(path) == filename:
            assets.assets[category] = [path]
            break

    xml = assets.model_xml(name, category, x, y, z, yaw, collision_xml)
    assets.assets[category] = original_list
    return xml

def get_vehicle_yaw(target_angle):
    VEHICLE_NATIVE_YAW = math.pi / 2
    return target_angle - VEHICLE_NATIVE_YAW

class PlacementManager:
    def __init__(self):
        self.forbidden_zones = []
        self.occupied_zones = []

        sidewalk_w = SIDEWALK
        half_road = ROAD_WIDTH / 2
        forbidden_half = half_road + sidewalk_w

        # Horizontal roads
        for r in range(ROWS + 1):
            y = START_Y - SPACING/2 + r * SPACING
            self.forbidden_zones.append((-9999, 9999, y - forbidden_half, y + forbidden_half))

        # Vertical roads
        for c in range(COLS + 1):
            x = START_X - SPACING/2 + c * SPACING
            self.forbidden_zones.append((x - forbidden_half, x + forbidden_half, -9999, 9999))

    def calculate_aabb(self, x, y, size_x, size_y, yaw):
        cos_y = abs(math.cos(yaw))
        sin_y = abs(math.sin(yaw))
        half_x = (size_x * cos_y + size_y * sin_y) / 2
        half_y = (size_x * sin_y + size_y * cos_y) / 2
        return (x - half_x, x + half_x, y - half_y, y + half_y)

    def register(self, aabb):
        self.occupied_zones.append(aabb)

CITY_MANAGER = PlacementManager()

def deterministic_spawn(name, category, filename, x, y, yaw_deg=0.0):
    global OBJ_COUNTER
    meta = ASSET_METADATA.get(filename)
    if not meta:
        return ""

    yaw = math.radians(yaw_deg)

    size_x = meta["size_x"]
    size_y = meta["size_y"]

    off_x, off_y = XY_OFFSETS.get(filename, (0.0, 0.0))
    rot_off_x = off_x * math.cos(yaw) - off_y * math.sin(yaw)
    rot_off_y = off_x * math.sin(yaw) + off_y * math.cos(yaw)

    spawn_x = x + rot_off_x
    spawn_y = y + rot_off_y

    aabb = CITY_MANAGER.calculate_aabb(x, y, size_x, size_y, yaw)
    CITY_MANAGER.register(aabb)

    OBJ_COUNTER += 1
    obj_id = f"{name}_{OBJ_COUNTER}"

    z_offset = meta.get("z_offset", 0.0)
    col_z = 5.0 - z_offset
    collision_xml = f"""
      <collision name="collision">
        <pose>{-off_x:.4f} {-off_y:.4f} {col_z:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{size_x:.4f} {size_y:.4f} 10.0</size></box>
        </geometry>
      </collision>
"""
    pose = {"x": spawn_x, "y": spawn_y, "z": z_offset, "yaw": yaw}
    add_obstacle_metadata(obj_id, category, filename, pose, aabb)
    return spawn_specific(obj_id, filename, category, spawn_x, spawn_y, yaw, 0.0, collision_xml)

def get_obstacle_cells():
    cells = set()
    for obs in WORLD_METADATA["obstacles"]:
        aabb = obs["aabb"]
        for gx in range(25):
            for gy in range(25):
                cell_min_x = gx * 4.0
                cell_max_x = (gx + 1) * 4.0
                cell_min_y = gy * 4.0
                cell_max_y = (gy + 1) * 4.0

                intersect_x = (aabb["max_x"] > cell_min_x) and (aabb["min_x"] < cell_max_x)
                intersect_y = (aabb["max_y"] > cell_min_y) and (aabb["min_y"] < cell_max_y)

                if intersect_x and intersect_y:
                    cells.add((gx, gy))
    # Reserve drone base cells (all 6 spawn positions)
    for spawn in WORLD_METADATA.get("drone_base", {}).get("spawns", []):
        sx = int(spawn["x"] // 4.0)
        sy = int(spawn["y"] // 4.0)
        cells.add((sx, sy))
    # Also reserve the drone base center and adjacent cells
    cells.update([(5, 6), (6, 6), (7, 6)])
    return cells

def spawn_victim(ideal_x, ideal_y):
    VICTIM_INTENTS.append((ideal_x, ideal_y))
    return ""

def _farthest_point_sampling(valid_cells_list, count, rng):
    """Randomized Farthest-Point Sampling for spatially dispersed victim placement.

    Algorithm:
    1. Select a seeded random initial cell.
    2. For each subsequent selection, compute the minimum Chebyshev distance from
       each candidate to all already-selected cells.
    3. Among candidates within 90% of the maximum min-distance, randomly select one.
       This introduces controlled randomness while preserving spatial dispersion.
    4. Continue until 'count' cells are selected.

    This guarantees reproducibility with a given seed and produces well-dispersed
    placements across the valid search area.
    """
    if count >= len(valid_cells_list):
        return valid_cells_list[:count]

    selected = []
    candidates = list(valid_cells_list)

    # Step 1: Random initial seed point
    first = rng.choice(candidates)
    selected.append(first)
    candidates.remove(first)

    # Steps 2-4: Iterative farthest-point selection with controlled randomness
    for _ in range(count - 1):
        # Compute minimum distance from each candidate to any selected point
        scored = []
        for c in candidates:
            min_dist = min(
                max(abs(c[0] - s[0]), abs(c[1] - s[1]))  # Chebyshev distance
                for s in selected
            )
            scored.append((c, min_dist))

        # Find the maximum min-distance
        max_min_dist = max(d for _, d in scored)

        # Select randomly among candidates within 90% of the max min-distance
        # This prevents deterministic tie-breaking while maintaining dispersion
        threshold = max_min_dist * 0.9
        top_candidates = [c for c, d in scored if d >= threshold]

        choice = rng.choice(top_candidates)
        selected.append(choice)
        candidates.remove(choice)

    return selected

def finalize_victims():
    """Places victims using Randomized Farthest-Point Sampling over valid free cells.

    Victim count and seed are controlled by set_victim_config() called from
    generate_world.py's CLI arguments (--victims, --seed).
    """
    xml = ""
    global OBJ_COUNTER
    obs_cells = get_obstacle_cells()

    # Build the set of all valid free cells (not obstacles, not drone spawns)
    all_cells = set()
    for gx in range(25):
        for gy in range(25):
            if (gx, gy) not in obs_cells:
                all_cells.add((gx, gy))

    valid_cells_list = sorted(all_cells)  # Sorted for determinism before RNG

    victim_count = _VICTIM_COUNT
    victim_rng = random.Random(_VICTIM_SEED)

    if victim_count > len(valid_cells_list):
        raise RuntimeError(
            f"Requested {victim_count} victims but only {len(valid_cells_list)} valid free cells exist."
        )

    # Use Randomized Farthest-Point Sampling for spatial dispersion
    selected_cells = _farthest_point_sampling(valid_cells_list, victim_count, victim_rng)

    for cell in selected_cells:
        OBJ_COUNTER += 1
        vid = f"victim_{OBJ_COUNTER}"
        # Place at center of the selected grid cell
        valid_x = cell[0] * 4.0 + 2.0
        valid_y = cell[1] * 4.0 + 2.0

        add_victim_metadata(vid, valid_x, valid_y)
        xml += f"""
  <include>
    <name>{vid}</name>
    <uri>model://rescue_randy_standing</uri>
    <pose>{valid_x:.2f} {valid_y:.2f} 0.0 0 0 0</pose>
  </include>
"""
    return xml

# ==========================================================
# Curated District Designs
# ==========================================================

def drone_base():
    cx, cy = 25.0, 25.0
    aabb = CITY_MANAGER.calculate_aabb(cx, cy, 14, 14, 0)
    CITY_MANAGER.register(aabb)

    WORLD_METADATA["drone_base"] = {
        "center_x": cx,
        "center_y": cy,
        "landing_zone_x": cx,
        "landing_zone_y": cy,
        "spawns": [
            {"id": "drone_0", "x": 21.0, "y": cy},
            {"id": "drone_1", "x": 29.0, "y": cy},
            {"id": "drone_2", "x": 25.0, "y": cy - 4.0},
            {"id": "drone_3", "x": 25.0, "y": cy + 4.0},
            {"id": "drone_4", "x": 21.0, "y": cy - 4.0},
            {"id": "drone_5", "x": 29.0, "y": cy + 4.0}
        ]
    }

    return f"""
  <model name="landing_pad_0_0">
    <static>true</static>
    <pose>{cx:.2f} {cy:.2f} 0.05 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>10 5 0.1</size></box></geometry>
        <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
"""

def residential():
    xml = ""
    xml += deterministic_spawn("res_house", "Houses", "house.glb", 50, 25, 0)
    xml += deterministic_spawn("res_rubble", "Rubble", "brick_rubble_scaniverse_lidar.glb", 50, 16, 0)
    return xml

def commercial():
    xml = ""
    xml += deterministic_spawn("comm_anchor", "Buildings", "futuristic_building.glb", 75, 27, 0)
    xml += deterministic_spawn("comm_car", "Cars", "small_price_car.glb", 70, 17, 90)
    xml += deterministic_spawn("comm_rubble", "Rubble", "construction_rubble.glb", 80, 17, 0)
    return xml

def park():
    xml = ""
    xml += deterministic_spawn("park_tower", "Towers", "40_meter_radiotower.glb", 18, 58, 0)
    xml += deterministic_spawn("park_rubble1", "Rubble", "construction_rubble.glb", 30, 45, 0)
    xml += deterministic_spawn("park_rubble2", "Rubble", "construction_rubble.glb", 22, 42, 0)
    return xml

def major_damage():
    xml = ""
    xml += deterministic_spawn("maj_bldg", "Buildings", "abandoned_building.glb", 50, 52, 0)
    xml += deterministic_spawn("maj_rubble", "Rubble", "brick_rubble_scaniverse_lidar.glb", 50, 42, 0)
    xml += deterministic_spawn("maj_car", "Cars", "old_rusty_car.glb", 45, 43, 45)
    return xml

def civic():
    xml = ""
    xml += deterministic_spawn("civ_apt", "Buildings", "low_poly_-_soviet__apartment_building_8k.glb", 70, 50, 0)
    xml += deterministic_spawn("civ_house", "Houses", "small_house.glb", 82, 55, 0)
    xml += deterministic_spawn("civ_rubble", "Rubble", "construction_rubble.glb", 80, 45, 0)
    return xml

def industrial():
    xml = ""
    xml += deterministic_spawn("ind_bldg", "Buildings", "abandoned_construction_building.glb", 25, 75, 0)
    xml += deterministic_spawn("ind_rubble", "Rubble", "brick_rubble_scaniverse_lidar.glb", 18, 75, 90)
    return xml

def mixed():
    xml = ""
    xml += deterministic_spawn("mix_bldg", "Buildings", "abandoned_building.glb", 50, 75, 0)
    xml += deterministic_spawn("mix_rubble", "Rubble", "construction_rubble.glb", 45, 82, 0)
    return xml

def collapsed():
    xml = ""
    xml += deterministic_spawn("col_apt", "Buildings", "low_poly_-_soviet__apartment_building_8k.glb", 75, 80, 90)
    xml += deterministic_spawn("col_rubble1", "Rubble", "brick_rubble_scaniverse_lidar.glb", 75, 68, 0)
    xml += deterministic_spawn("col_rubble2", "Rubble", "construction_rubble.glb", 68, 68, 0)
    return xml

# ==========================================================
# Main Generator
# ==========================================================

def generate_city():
    xml = ""
    global CITY_MANAGER, WORLD_METADATA, OBJ_COUNTER, VICTIM_INTENTS
    CITY_MANAGER = PlacementManager()
    WORLD_METADATA = {"obstacles": [], "victims": [], "drone_base": {}}
    OBJ_COUNTER = 0
    VICTIM_INTENTS = []

    xml += drone_base()
    xml += residential()
    xml += commercial()
    xml += park()
    xml += major_damage()
    xml += civic()
    xml += industrial()
    xml += mixed()
    xml += collapsed()

    xml += finalize_victims()

    return xml, WORLD_METADATA