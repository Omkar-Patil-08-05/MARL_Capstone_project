import os
import math
import json
from forest_config import (
    WORLD_SIZE, RNG, BASE_SIZE, BASE_CLEARING_RADIUS, SETTLEMENT_SIZE,
    DEFAULT_TREE_COUNT, DEFAULT_UNDERSTORY_COUNT, DEFAULT_LOG_COUNT, DEFAULT_ROCK_COUNT,
    DEFAULT_CAST_SHADOWS, VICTIM_GROUND_OFFSET, ASSET_ROOT, SEARCH_REGION_SIZE
)
from forest_terrain import generate_forest_heightmap, get_terrain_sdf, ensure_terrain_textures
from forest_layout import ForestLayoutManager
from assets import AssetLibrary, ASSET_METADATA

assets = AssetLibrary()

def spawn_real_pine(name, x, y, z, yaw, scale=1.2, give_collision=True):
    col_xml = ""
    if give_collision:
        trunk_h = 7.0 * scale
        trunk_r = 0.35 * scale
        col_xml = f"""
      <collision name="trunk_collision">
        <pose>0 0 {trunk_h / 2.0:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{trunk_r:.3f}</radius><length>{trunk_h:.3f}</length></cylinder></geometry>
      </collision>"""
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>
    <link name="link">{col_xml}
      <visual name="branch">
        <geometry>
          <mesh>
            <uri>file://{ASSET_ROOT}/Trees/pine_tree/meshes/pine_tree.dae</uri>
            <submesh><name>Branch</name></submesh>
            <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>
          </mesh>
        </geometry>
        <material>
          <double_sided>true</double_sided>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>file://{ASSET_ROOT}/Trees/pine_tree/materials/textures/branch_2_diffuse.png</albedo_map></metal></pbr>
        </material>
      </visual>
      <visual name="bark">
        <geometry>
          <mesh>
            <uri>file://{ASSET_ROOT}/Trees/pine_tree/meshes/pine_tree.dae</uri>
            <submesh><name>Bark</name></submesh>
            <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>
          </mesh>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>file://{ASSET_ROOT}/Trees/pine_tree/materials/textures/bark_diffuse.png</albedo_map></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
"""

def spawn_real_oak(name, x, y, z, yaw, scale=1.0, give_collision=True):
    col_xml = ""
    if give_collision:
        trunk_h = 5.5 * scale
        trunk_r = 0.42 * scale
        col_xml = f"""
      <collision name="trunk_collision">
        <pose>0 0 {trunk_h / 2.0:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{trunk_r:.3f}</radius><length>{trunk_h:.3f}</length></cylinder></geometry>
      </collision>"""
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>
    <link name="link">{col_xml}
      <visual name="branch">
        <geometry>
          <mesh>
            <uri>file://{ASSET_ROOT}/Trees/oak_tree/meshes/oak_tree.dae</uri>
            <submesh><name>Branch</name></submesh>
            <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>
          </mesh>
        </geometry>
        <material>
          <double_sided>true</double_sided>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>file://{ASSET_ROOT}/Trees/oak_tree/materials/textures/branch_diffuse.png</albedo_map></metal></pbr>
        </material>
      </visual>
      <visual name="bark">
        <geometry>
          <mesh>
            <uri>file://{ASSET_ROOT}/Trees/oak_tree/meshes/oak_tree.dae</uri>
            <submesh><name>Bark</name></submesh>
            <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>
          </mesh>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>file://{ASSET_ROOT}/Trees/oak_tree/materials/textures/bark_diffuse.png</albedo_map></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
"""

def spawn_glb_nature(name, glb_file, x, y, z, yaw, scale=1.0, col_r=0.0, col_h=0.0):
    col_xml = ""
    if col_r > 0 and col_h > 0:
        col_xml = f"""
      <collision name="collision">
        <pose>0 0 {col_h / 2.0:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{col_r:.3f}</radius><length>{col_h:.3f}</length></cylinder></geometry>
      </collision>"""
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x:.2f} {y:.2f} {z:.2f} 1.5708 0 {yaw:.4f}</pose>
    <link name="link">{col_xml}
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>file://{ASSET_ROOT}/Trees/{glb_file}</uri>
            <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
"""

def generate_base(layout: ForestLayoutManager, metadata: dict):
    """
    Generates the SAR Drone Staging Base Camp:
    - 6 designated drone launch pads (base_pad_0 .. base_pad_5)
    - Central Operations HQ Building (futuristic_building.glb)
    - 40-meter Radio Communications Tower
    - 2 Forward Operating emergency tents (tent_detailedOpen.glb)
    """
    xml = ""
    cx, cy = layout.base_world_x, layout.base_world_y
    cz = layout.get_height(cx, cy)

    # 6 drone landing pads around the operations center
    pad_offsets = [(-18.0, -10.0), (18.0, -10.0), (-18.0, 10.0), (18.0, 10.0), (0.0, -20.0), (0.0, 20.0)]
    spawns = []

    for i, (dx, dy) in enumerate(pad_offsets):
        px, py = cx + dx, cy + dy
        xml += f"""
  <model name="base_pad_{i}">
    <static>true</static>
    <pose>{px:.2f} {py:.2f} {cz + 0.10:.2f} 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>5 5 0.2</size></box></geometry>
        <material><ambient>0.95 0.50 0.10 1</ambient><diffuse>0.95 0.50 0.10 1</diffuse></material>
      </visual>
    </link>
  </model>
"""
        spawns.append({
            "id": f"drone_{i}",
            "x": float(px),
            "y": float(py),
            "z": float(cz + 0.22)
        })

    # Central Ops Building
    try:
        filename = "futuristic_building.glb"
        meta = ASSET_METADATA.get(filename, {"size_x": 21.89, "size_y": 15.00, "z_offset": -5.10})
        z_off = meta.get("z_offset", 0.0)
        col_z = 5.0 - z_off
        collision_xml = f"""
      <collision name="collision">
        <pose>0 0 {col_z:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{meta["size_x"]:.4f} {meta["size_y"]:.4f} 10.0</size></box>
        </geometry>
      </collision>
"""
        original_list = assets.assets.get("Buildings", [])
        try:
            for path in original_list:
                if os.path.basename(path) == filename:
                    assets.assets["Buildings"] = [path]
                    xml += assets.model_xml("base_ops_building", "Buildings", cx, cy, cz, 0.0, collision_xml)
                    metadata.setdefault("obstacles", []).append({
                        "id": "base_ops_building",
                        "type": "building",
                        "asset": filename,
                        "pose": {"x": cx, "y": cy, "z": cz, "yaw": 0.0},
                        "aabb": {
                            "min_x": cx - meta["size_x"] / 2.0,
                            "max_x": cx + meta["size_x"] / 2.0,
                            "min_y": cy - meta["size_y"] / 2.0,
                            "max_y": cy + meta["size_y"] / 2.0
                        }
                    })
                    break
        finally:
            assets.assets["Buildings"] = original_list
    except Exception as e:
        print(f"Ops building placement failed: {e}")

    # Forward Operating SAR Tents on the base perimeter
    tent_placements = [
        ("base_tent_1", cx - 22.0, cy - 2.0, 0.35),
        ("base_tent_2", cx + 22.0, cy - 2.0, -0.35)
    ]
    for t_name, tx, ty, tyaw in tent_placements:
        xml += spawn_glb_nature(t_name, "tent_detailedOpen.glb", tx, ty, cz, tyaw, scale=4.0, col_r=2.5, col_h=2.8)

    metadata["drone_base"] = {
        "center": {"x": cx, "y": cy, "z": cz},
        "spawns": spawns
    }
    return xml

def generate_settlement(layout: ForestLayoutManager, metadata: dict):
    xml = ""
    cx, cy = layout.settle_world_x, layout.settle_world_y

    placements = [
        ("house_1", "Houses", "house.glb", cx - 15, cy - 10, 0),
        ("house_2", "Houses", "small_house.glb", cx + 15, cy + 5, 90),
        ("house_3", "Houses", "small_house.glb", cx, cy + 15, 180),
        ("rubble_1", "Rubble", "brick_rubble_scaniverse_lidar.glb", cx - 5, cy - 5, 45),
        ("rubble_2", "Rubble", "construction_rubble.glb", cx + 5, cy - 15, 0),
        ("car_1", "Cars", "old_rusty_car.glb", cx + 10, cy - 5, 30),
    ]

    metadata["settlement"] = {"center": {"x": cx, "y": cy, "z": layout.get_height(cx, cy)}, "buildings": []}

    for name, cat, fname, px, py, yaw_deg in placements:
        pz = layout.get_height(px, py)
        meta = ASSET_METADATA.get(fname)
        if meta:
            yaw = math.radians(yaw_deg)
            z_off = meta.get("z_offset", 0.0)
            col_z = 5.0 - z_off
            collision_xml = f"""
      <collision name="collision">
        <pose>0 0 {col_z:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{meta["size_x"]:.4f} {meta["size_y"]:.4f} 10.0</size></box>
        </geometry>
      </collision>
"""
            orig_list = assets.assets.get(cat, [])
            try:
                found = False
                for path in orig_list:
                    if os.path.basename(path) == fname:
                        assets.assets[cat] = [path]
                        xml += assets.model_xml(name, cat, px, py, pz, yaw, collision_xml)
                        metadata["settlement"]["buildings"].append({"id": name, "x": px, "y": py, "z": pz})
                        metadata.setdefault("obstacles", []).append({
                            "id": name,
                            "type": cat.lower(),
                            "asset": fname,
                            "pose": {"x": px, "y": py, "z": pz, "yaw": yaw},
                            "aabb": {
                                "min_x": px - meta["size_x"] / 2.0,
                                "max_x": px + meta["size_x"] / 2.0,
                                "min_y": py - meta["size_y"] / 2.0,
                                "max_y": py + meta["size_y"] / 2.0
                            }
                        })
                        found = True
                        break
            finally:
                assets.assets[cat] = orig_list
    return xml

def generate_forest_world(seed, victims_count, out_sdf, out_map, out_heightmap, out_meta,
                          tree_count=DEFAULT_TREE_COUNT,
                          cast_shadows=DEFAULT_CAST_SHADOWS,
                          victim_ground_offset=VICTIM_GROUND_OFFSET):
    """
    Generates a realistic, asset-based MARL Forest SAR simulation world:
    - 500m x 500m uneven terrain heightmap with winding river corridor and mountain ridges.
    - Clustered stands of authentic 3D tree models (Open Robotics Pine, Open Robotics Oak, Conifers).
    - Understory shrubs, fallen dead logs/stumps, riverbank boulders.
    - 6-drone SAR staging base camp at (-150, 150) with HQ building and emergency tents.
    - Distributed search sectors with grounded victims and thermal signatures.
    - Full Gazebo Harmonic ODE physics and georeferencing setup for PX4 SITL lockstep.
    """
    print(f"\n=======================================================")
    print(f"Generating Asset-Based Forest World (seed={seed}, trees={tree_count}, victims={victims_count})")
    print(f"=======================================================")

    # 1. Heightmap generation
    terrain_map, max_h = generate_forest_heightmap(seed, out_heightmap)

    # 2. Layout manager setup
    layout = ForestLayoutManager(terrain_map, max_height=max_h)

    metadata = {
        "seed": seed,
        "world_size": WORLD_SIZE,
        "max_height": max_h,
        "tree_count": tree_count,
        "cast_shadows": cast_shadows,
        "victim_ground_offset": victim_ground_offset,
        "asset_library": {
            "pine_tree": {"source": "Open Robotics / Gazebo Fuel", "format": "DAE + PBR", "license": "CC-BY 4.0"},
            "oak_tree": {"source": "Open Robotics / Gazebo Fuel", "format": "DAE + PBR", "license": "CC-BY 4.0"},
            "pine_tall": {"source": "Kenney Nature Kit", "format": "GLB", "license": "CC0 Public Domain"},
            "broadleaf_dense": {"source": "Kenney Nature Kit", "format": "GLB", "license": "CC0 Public Domain"},
            "understory": {"source": "Kenney Nature Kit", "format": "GLB", "license": "CC0 Public Domain"},
            "fallen_logs": {"source": "Kenney Nature Kit", "format": "GLB", "license": "CC0 Public Domain"},
            "river_rocks": {"source": "Kenney Nature Kit", "format": "GLB", "license": "CC0 Public Domain"}
        },
        "terrain": {
            "size_m": WORLD_SIZE,
            "width_m": WORLD_SIZE,
            "height_m": WORLD_SIZE,
            "max_height_m": max_h,
            "origin_x": -WORLD_SIZE / 2.0,
            "origin_y": -WORLD_SIZE / 2.0
        },
        "search_area": {
            "size_m": SEARCH_REGION_SIZE,
            "width_m": SEARCH_REGION_SIZE,
            "height_m": SEARCH_REGION_SIZE,
            "origin_x": -180.0,
            "origin_y": -20.0,
            "grid_width": 25,
            "grid_height": 25,
            "meters_per_cell": 8.0,
            "description": "200m x 200m operational SAR grid spanning staging base, river, and forested ridges"
        },
        "sectors": [
            {"id": "sector_1", "name": "Sector 1 (Base Staging)", "center": {"x": -150.0, "y": 150.0}, "radius": 35.0},
            {"id": "sector_2", "name": "Sector 2 (River Corridor)", "center": {"x": -40.0, "y": 60.0}, "radius": 55.0},
            {"id": "sector_3", "name": "Sector 3 (Northwest Ridge)", "center": {"x": -85.0, "y": 120.0}, "radius": 45.0},
            {"id": "sector_4", "name": "Sector 4 (East Woodland)", "center": {"x": 20.0, "y": 80.0}, "radius": 50.0}
        ],
        "obstacles": [],
        "victims": [],
        "victims_requested": victims_count,
        "victims_generated": 0
    }

    # 3. Terrain SDF with 512x512 diffuse and normal textures
    abs_heightmap = os.path.abspath(out_heightmap)
    out_dir = os.path.dirname(os.path.abspath(out_sdf)) if out_sdf else os.path.dirname(abs_heightmap)
    diffuse_path, normal_path = ensure_terrain_textures(out_dir)
    sdf_xml = get_terrain_sdf(abs_heightmap, diffuse_path, normal_path, max_height=max_h)

    # 4. Base Camp & Settlement
    sdf_xml += generate_base(layout, metadata)
    sdf_xml += generate_settlement(layout, metadata)

    # 5. Clustered Tree Generation (Real 3D Assets)
    trees = layout.generate_clustered_trees(total_count=tree_count)
    trees_with_collision = 0
    trees_visual_only = 0
    tree_visuals_list = []

    for i, t in enumerate(trees):
        tree_name = f"tree_{i}"
        species = t["species"]
        tx, ty, tz = t["x"], t["y"], t["z"]
        yaw = t["yaw"]
        scale = t["scale"]

        # Give collision to operational trees inside the active search sector
        in_search = (-180.0 <= tx <= 20.0) and (-20.0 <= ty <= 180.0)
        give_col = bool(in_search and (trees_with_collision < 80))

        if species == "pine_tree":
            sdf_xml += spawn_real_pine(tree_name, tx, ty, tz, yaw, scale=scale, give_collision=give_col)
        elif species == "oak_tree":
            sdf_xml += spawn_real_oak(tree_name, tx, ty, tz, yaw, scale=scale, give_collision=give_col)
        elif species == "pine_tall":
            glb_choice = "pine_tall_a.glb" if (i % 2 == 0) else "pine_tall_b.glb"
            col_r = 0.35 * scale / 6.0 if give_col else 0.0
            col_h = 9.0 * scale / 6.0 if give_col else 0.0
            sdf_xml += spawn_glb_nature(tree_name, glb_choice, tx, ty, tz, yaw, scale=scale, col_r=col_r, col_h=col_h)
        else: # broadleaf_dense
            glb_choice = "detailed_tree.glb" if (i % 2 == 0) else "oak_tree.glb"
            col_r = 0.40 * scale / 6.0 if give_col else 0.0
            col_h = 7.0 * scale / 6.0 if give_col else 0.0
            sdf_xml += spawn_glb_nature(tree_name, glb_choice, tx, ty, tz, yaw, scale=scale, col_r=col_r, col_h=col_h)

        if give_col:
            trees_with_collision += 1
            metadata["obstacles"].append({
                "id": tree_name,
                "type": "tree",
                "species": species,
                "pose": {"x": float(tx), "y": float(ty), "z": float(tz), "yaw": float(yaw)},
                "aabb": {
                    "min_x": float(tx - t["canopy_radius"]),
                    "max_x": float(tx + t["canopy_radius"]),
                    "min_y": float(ty - t["canopy_radius"]),
                    "max_y": float(ty + t["canopy_radius"])
                }
            })
        else:
            trees_visual_only += 1

        tree_visuals_list.append({
            "id": tree_name,
            "species": species,
            "x": round(float(tx), 2),
            "y": round(float(ty), 2),
            "z": round(float(tz), 2),
            "scale": round(float(scale), 2),
            "has_collision": bool(give_col)
        })

    print(f"  Trees placed: {len(trees)} ({trees_with_collision} with trunk collision, {trees_visual_only} visual-only)")

    metadata["trees"] = {
        "total": len(trees),
        "with_collision": trees_with_collision,
        "visual_only": trees_visual_only,
        "species_breakdown": {
            "openrobotics_pine": sum(1 for t in trees if t["species"] == "pine_tree"),
            "openrobotics_oak": sum(1 for t in trees if t["species"] == "oak_tree"),
            "kenney_pine_tall": sum(1 for t in trees if t["species"] == "pine_tall"),
            "kenney_broadleaf": sum(1 for t in trees if t["species"] == "broadleaf_dense")
        }
    }
    metadata["tree_visuals"] = tree_visuals_list

    # 6. Understory Vegetation (Bushes and Shrubs)
    bushes = layout.generate_understory_bushes(count=DEFAULT_UNDERSTORY_COUNT)
    for b_i, b in enumerate(bushes):
        sdf_xml += spawn_glb_nature(f"bush_{b_i}", f"{b['model']}.glb", b["x"], b["y"], b["z"], b["yaw"], scale=b["scale"])
    print(f"  Understory shrubs placed: {len(bushes)}")

    # 7. Fallen Logs and Dead Stumps
    logs = layout.generate_fallen_logs(count=DEFAULT_LOG_COUNT)
    for l_i, l in enumerate(logs):
        sdf_xml += spawn_glb_nature(f"log_{l_i}", f"{l['model']}.glb", l["x"], l["y"], l["z"], l["yaw"], scale=l["scale"], col_r=0.45, col_h=0.7)
    print(f"  Fallen logs / stumps placed: {len(logs)}")

    # 8. Riverbank Boulders and Rocks
    rocks = layout.generate_riverbank_rocks(count=DEFAULT_ROCK_COUNT)
    for r_i, r in enumerate(rocks):
        sdf_xml += spawn_glb_nature(f"rock_{r_i}", "stone_large.glb", r["x"], r["y"], r["z"], r["yaw"], scale=r["scale"], col_r=1.2, col_h=1.5)
    print(f"  Riverbank rocks placed: {len(rocks)}")

    # 9. Victims Generation
    victims = layout.generate_victims(count=victims_count)
    for v in victims:
        vid = f"victim_{v['id']}"
        gx = min(24, max(0, int(math.floor((v["x"] - (-180.0)) / 8.0))))
        gy = min(24, max(0, int(math.floor((v["y"] - (-20.0)) / 8.0))))

        metadata["victims"].append({
            "id": vid,
            "sector": v["sector"],
            "grid": {"x": gx, "y": gy},
            "world": {"x": v["x"], "y": v["y"], "z": v["z"]},
            "world_x": v["x"],
            "world_y": v["y"],
            "world_z": v["z"],
            "terrain_elevation": v["ground_z"],
            "ground_offset": victim_ground_offset,
            "slope_deg": v.get("slope_deg", 0.0),
            "model": v["model"]
        })
        sdf_xml += f"""
  <include>
    <name>{vid}</name>
    <uri>model://{v["model"]}</uri>
    <pose>{v["x"]:.2f} {v["y"]:.2f} {v["z"]:.2f} 0 0 {v["yaw"]:.4f}</pose>
  </include>
"""
    metadata["victims_generated"] = len(victims)
    print(f"  Victims placed: {len(victims)}/{victims_count}")

    # 10. Assemble Full World SDF
    world_name = os.path.splitext(os.path.basename(out_sdf))[0] if out_sdf else "forest_sar"
    shadows_str = "true" if cast_shadows else "false"

    full_sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
<world name="{world_name}">

  <!-- World-level system plugins — self-contained, no server.config needed -->
  <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system"           name="gz::sim::systems::Contact"/>
  <plugin filename="gz-sim-imu-system"               name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-air-pressure-system"      name="gz::sim::systems::AirPressure"/>
  <plugin filename="gz-sim-air-speed-system"         name="gz::sim::systems::AirSpeed"/>
  <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
  <plugin filename="gz-sim-navsat-system"            name="gz::sim::systems::NavSat"/>
  <plugin filename="gz-sim-magnetometer-system"      name="gz::sim::systems::Magnetometer"/>
  <plugin filename="gz-sim-sensors-system"           name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>

  <physics type="ode">
    <max_step_size>0.004</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>250</real_time_update_rate>
    <ode>
      <solver>
        <type>quick</type>
        <iters>12</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
  <gravity>0 0 -9.8</gravity>
  <atmosphere type="adiabatic"/>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>

  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg>
    <elevation>12.0</elevation>
  </spherical_coordinates>

  <scene>
    <ambient>0.65 0.65 0.65 1</ambient>
    <background>0.72 0.84 0.97 1</background>
    <grid>false</grid>
    <origin_visual>false</origin_visual>
  </scene>

  <light name="sun" type="directional">
    <cast_shadows>{shadows_str}</cast_shadows>
    <pose>0 0 200 0 0 0</pose>
    <diffuse>1 1 1 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.4 0.2 -1</direction>
  </light>

{sdf_xml}

</world>
</sdf>
"""

    with open(out_sdf, "w") as f:
        f.write(full_sdf)

    layout.render_2d_map(trees, victims, out_map)

    with open(out_meta, "w") as f:
        json.dump(metadata, f, indent=4)

    print(f"  SDF written: {out_sdf} ({os.path.getsize(out_sdf):,} bytes)")
    print(f"  2D Map written: {out_map}")
    print(f"  Metadata written: {out_meta}")
    return metadata
