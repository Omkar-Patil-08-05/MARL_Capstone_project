import bpy
import os
import sys
import math
from mathutils import Vector

def get_bbox(obj, matrix):
    corners = [matrix @ Vector(corner) for corner in obj.bound_box]
    min_x = min([c.x for c in corners])
    min_y = min([c.y for c in corners])
    min_z = min([c.z for c in corners])
    max_x = max([c.x for c in corners])
    max_y = max([c.y for c in corners])
    max_z = max([c.z for c in corners])
    return (min_x, min_y, min_z), (max_x, max_y, max_z)

def analyze_glb(filepath):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=filepath)
    
    meshes = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    if not meshes:
        return None
        
    global_min_x = float('inf')
    global_min_y = float('inf')
    global_min_z = float('inf')
    global_max_x = float('-inf')
    global_max_y = float('-inf')
    global_max_z = float('-inf')
    
    for obj in meshes:
        min_v, max_v = get_bbox(obj, obj.matrix_world)
        global_min_x = min(global_min_x, min_v[0])
        global_min_y = min(global_min_y, min_v[1])
        global_min_z = min(global_min_z, min_v[2])
        global_max_x = max(global_max_x, max_v[0])
        global_max_y = max(global_max_y, max_v[1])
        global_max_z = max(global_max_z, max_v[2])
        
    return {
        "min": (global_min_x, global_min_y, global_min_z),
        "max": (global_max_x, global_max_y, global_max_z),
        "size": (global_max_x - global_min_x, global_max_y - global_min_y, global_max_z - global_min_z)
    }

assets = [
    "Houses/house.glb",
    "Houses/small_house.glb",
    "Buildings/abandoned_building.glb",
    "Buildings/abandoned_construction_building.glb",
    "Buildings/futuristic_building.glb",
    "Buildings/low_poly_-_soviet__apartment_building_8k.glb",
    "Buildings/post-apocalyptic_buildings.glb",
    "Buildings/residential_complex_modern_apartment_building.glb",
    "Cars/old_rusty_car.glb",
    "Cars/small_price_car.glb",
    "Rubble/brick_rubble_scaniverse_lidar.glb",
    "Rubble/construction_rubble.glb",
    "Towers/40_meter_radiotower.glb"
]

asset_root = "/home/capstone/capstone_project_antigravity/assets_real"
for asset in assets:
    full_path = os.path.join(asset_root, asset)
    if os.path.exists(full_path):
        res = analyze_glb(full_path)
        if res:
            print(f"ASSET: {asset}")
            print(f"  Min: {res['min'][0]:.4f}, {res['min'][1]:.4f}, {res['min'][2]:.4f}")
            print(f"  Max: {res['max'][0]:.4f}, {res['max'][1]:.4f}, {res['max'][2]:.4f}")
            print(f"  Size: {res['size'][0]:.4f}, {res['size'][1]:.4f}, {res['size'][2]:.4f}")
