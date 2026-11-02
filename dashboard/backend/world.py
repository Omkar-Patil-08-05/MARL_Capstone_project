import json
import os

MAP_REGISTRY = {
    "realistic_sar": {
        "id": "realistic_sar",
        "name": "Urban Disaster Theater (Earthquake City)",
        "world_file": "realistic_sar.sdf",
        "metadata_file": "generated_world_meta.json",
        "grid_width": 25,
        "grid_height": 25,
        "meters_per_cell": 4.0,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "victim_count": 5,
        "policy_compatible": True
    },
    "forest_sar": {
        "id": "forest_sar",
        "name": "Wilderness SAR Sector (Mountain Forest)",
        "world_file": "forest_sar.sdf",
        "metadata_file": "forest_sar_meta.json",
        "grid_width": 25,
        "grid_height": 25,
        "meters_per_cell": 8.0,
        "origin_x": -180.0,
        "origin_y": -20.0,
        "terrain_width_m": 500.0,
        "terrain_height_m": 500.0,
        "terrain_origin_x": -250.0,
        "terrain_origin_y": -250.0,
        "victim_count": 5,
        "policy_compatible": True
    },
    "forest_dense_candidate": {
        "id": "forest_dense_candidate",
        "name": "Dense Wilderness Sector (Dense Woodland / 950 Trees)",
        "world_file": "forest_dense_candidate.sdf",
        "metadata_file": "forest_dense_candidate_meta.json",
        "grid_width": 25,
        "grid_height": 25,
        "meters_per_cell": 8.0,
        "origin_x": -180.0,
        "origin_y": -20.0,
        "terrain_width_m": 500.0,
        "terrain_height_m": 500.0,
        "terrain_origin_x": -250.0,
        "terrain_origin_y": -250.0,
        "victim_count": 5,
        "policy_compatible": True
    }
}

def get_map_registry():
    registry = dict(MAP_REGISTRY)
    for map_id, entry in registry.items():
        meta_filename = entry.get("metadata_file")
        if meta_filename:
            meta_path = os.path.expanduser(f"~/capstone_project_antigravity/worlds/{meta_filename}")
            if os.path.exists(meta_path):
                try:
                    with open(meta_path, 'r') as f:
                        meta = json.load(f)
                    actual_count = meta.get("victims_requested", len(meta.get("victims", [])))
                    if actual_count:
                        entry_copy = dict(entry)
                        entry_copy["victim_count"] = actual_count
                        registry[map_id] = entry_copy
                except Exception:
                    pass
    return registry

def get_world_data(map_id: str):
    if map_id not in MAP_REGISTRY:
        return {"error": "Map not found in registry."}
        
    registry_entry = MAP_REGISTRY[map_id]
    meta_filename = registry_entry["metadata_file"]
    meta_path = os.path.expanduser(f"~/capstone_project_antigravity/worlds/{meta_filename}")
    
    try:
        with open(meta_path, 'r') as f:
            meta = json.load(f)
            
        victims_list = meta.get("victims", [])
        actual_victim_count = meta.get("victims_requested", len(victims_list))

        world_data = {
            "id": registry_entry["id"],
            "name": registry_entry["name"],
            "victim_count": actual_victim_count,
            "grid": {
                "width": registry_entry["grid_width"],
                "height": registry_entry["grid_height"],
                "meters_per_cell": registry_entry["meters_per_cell"]
            },
            "world": {
                "width_m": registry_entry["grid_width"] * registry_entry["meters_per_cell"],
                "height_m": registry_entry["grid_height"] * registry_entry["meters_per_cell"],
                "origin_x": registry_entry.get("origin_x", 0.0),
                "origin_y": registry_entry.get("origin_y", 0.0),
                "terrain_width_m": registry_entry.get("terrain_width_m", registry_entry["grid_width"] * registry_entry["meters_per_cell"]),
                "terrain_height_m": registry_entry.get("terrain_height_m", registry_entry["grid_height"] * registry_entry["meters_per_cell"]),
                "terrain_origin_x": registry_entry.get("terrain_origin_x", registry_entry.get("origin_x", 0.0)),
                "terrain_origin_y": registry_entry.get("terrain_origin_y", registry_entry.get("origin_y", 0.0))
            },
            "obstacles": meta.get("obstacles", []),
            "victims": victims_list,
            "drone_spawns": meta.get("drone_base", {}).get("spawns", []),
            "drone_base": meta.get("drone_base", {}),
            "search_area": meta.get("search_area", {}),
            "policy_compatible": registry_entry["policy_compatible"],
            "sectors": meta.get("sectors", []),
            "tree_visuals": meta.get("tree_visuals", [])
        }
        return world_data
    except Exception as e:
        return {"error": str(e)}
