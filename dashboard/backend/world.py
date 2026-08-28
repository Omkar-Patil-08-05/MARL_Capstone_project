import json
import os

MAP_REGISTRY = {
    "realistic_sar": {
        "id": "realistic_sar",
        "name": "Earthquake City (Realistic SAR)",
        "world_file": "realistic_sar.sdf",
        "metadata_file": "generated_world_meta.json",
        "grid_width": 25,
        "grid_height": 25,
        "meters_per_cell": 4.0,
        "victim_count": 5,
        "policy_compatible": True
    }
}

def get_map_registry():
    return MAP_REGISTRY

def get_world_data(map_id: str):
    if map_id not in MAP_REGISTRY:
        return {"error": "Map not found in registry."}
        
    registry_entry = MAP_REGISTRY[map_id]
    meta_filename = registry_entry["metadata_file"]
    meta_path = os.path.expanduser(f"~/capstone_project_antigravity/worlds/{meta_filename}")
    
    try:
        with open(meta_path, 'r') as f:
            meta = json.load(f)
            
        world_data = {
            "id": registry_entry["id"],
            "name": registry_entry["name"],
            "grid": {
                "width": registry_entry["grid_width"],
                "height": registry_entry["grid_height"],
                "meters_per_cell": registry_entry["meters_per_cell"]
            },
            "world": {
                "width_m": registry_entry["grid_width"] * registry_entry["meters_per_cell"],
                "height_m": registry_entry["grid_height"] * registry_entry["meters_per_cell"]
            },
            "obstacles": meta.get("obstacles", []),
            "victims": meta.get("victims", []),
            "drone_spawns": meta.get("drone_base", {}).get("spawns", []),
            "policy_compatible": registry_entry["policy_compatible"]
        }
        return world_data
    except Exception as e:
        return {"error": str(e)}
