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

def add_obstacle_metadata(obj_id, category, aabb):
    WORLD_METADATA["obstacles"].append({
        "id": obj_id,
        "type": category,
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
def spawn_specific(name, filename, category, x, y, yaw=0.0, z=0.0):
    original_list = assets.assets.get(category, [])
    for path in original_list:
        if os.path.basename(path) == filename:
            assets.assets[category] = [path]
            break
    
    xml = assets.model_xml(name, category, x, y, z, yaw)
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
    add_obstacle_metadata(f"{name}_{OBJ_COUNTER}", category, aabb)
    return spawn_specific(name, filename, category, spawn_x, spawn_y, yaw)

def spawn_victim(ideal_x, ideal_y):
    global OBJ_COUNTER
    OBJ_COUNTER += 1
    vid = f"victim_{OBJ_COUNTER}"
    add_victim_metadata(vid, ideal_x, ideal_y)
    xml = f"""
  <model name="{vid}">
    <static>true</static>
    <pose>{ideal_x:.2f} {ideal_y:.2f} 0.25 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>0.25</radius></sphere></geometry>
        <material><ambient>1.0 0.2 0.2 1</ambient><diffuse>1.0 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
  </model>
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
            {"id": "drone_1", "x": 29.0, "y": cy}
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
    # Victim safe placement outside the rubble AABB
    xml += spawn_victim(62.0, 18.0)
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
    # Victim safe placement in open space
    xml += spawn_victim(26.0, 50.0)
    return xml

def major_damage():
    xml = ""
    xml += deterministic_spawn("maj_bldg", "Buildings", "abandoned_building.glb", 50, 52, 0)
    xml += deterministic_spawn("maj_rubble", "Rubble", "brick_rubble_scaniverse_lidar.glb", 50, 42, 0)
    xml += deterministic_spawn("maj_car", "Cars", "old_rusty_car.glb", 45, 43, 45)
    # Victim safe placement adjacent to rubble
    xml += spawn_victim(38.0, 42.0)
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
    # Victim safe placement outside building/rubble bounds
    xml += spawn_victim(26.0, 62.0)
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
    # Victim safe placement adjacent to major rubble
    xml += spawn_victim(62.0, 66.0)
    return xml

# ==========================================================
# Main Generator
# ==========================================================

def generate_city():
    xml = ""
    global CITY_MANAGER, WORLD_METADATA, OBJ_COUNTER
    CITY_MANAGER = PlacementManager()
    WORLD_METADATA = {"obstacles": [], "victims": [], "drone_base": {}}
    OBJ_COUNTER = 0
    
    xml += drone_base()
    xml += residential()
    xml += commercial()
    xml += park()
    xml += major_damage()
    xml += civic()
    xml += industrial()
    xml += mixed()
    xml += collapsed()
    
    return xml, WORLD_METADATA