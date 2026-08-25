import os
import sys

# Import assets metadata and AssetLibrary from assets.py
from assets import ASSET_METADATA, AssetLibrary

# Define the models we want to include in the validation world
VALIDATION_MODELS = [
    {"name": "validation_house", "filename": "house.glb", "category": "Houses"},
    {"name": "validation_small_house", "filename": "small_house.glb", "category": "Houses"},
    {"name": "validation_building", "filename": "futuristic_building.glb", "category": "Buildings"},
    {"name": "validation_rusty_car", "filename": "old_rusty_car.glb", "category": "Cars"},
    {"name": "validation_small_car", "filename": "small_price_car.glb", "category": "Cars"},
    {"name": "validation_brick_rubble", "filename": "brick_rubble_scaniverse_lidar.glb", "category": "Rubble"},
    {"name": "validation_construction_rubble", "filename": "construction_rubble.glb", "category": "Rubble"},
    {"name": "validation_tower", "filename": "40_meter_radiotower.glb", "category": "Towers"},
]

def generate_validation_world():
    world_xml = f"""<?xml version="1.0"?>
<sdf version="1.9">
<world name="validation_world">

<physics type="dart">
  <max_step_size>0.004</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>

<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

<scene>
  <ambient>0.6 0.6 0.6 1</ambient>
  <background>0.7 0.7 0.7 1</background>
  <shadows>false</shadows>
  <grid>false</grid>
</scene>

<light type="directional" name="sun">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.5 0.1 -0.9</direction>
</light>

<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>500 500</size>
        </plane>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>500 500</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.25 0.52 0.25 1</ambient>
        <diffuse>0.25 0.52 0.25 1</diffuse>
      </material>
    </visual>
  </link>
</model>

"""

    x = 0
    y = 0
    spacing = 40.0

    asset_root = "/home/capstone/capstone_project_antigravity/assets_real"

    for idx, model in enumerate(VALIDATION_MODELS):
        filename = model["filename"]
        if filename not in ASSET_METADATA:
            print(f"Warning: {filename} not found in ASSET_METADATA")
            continue
            
        meta = ASSET_METADATA[filename]
        roll, pitch, yaw = meta["rotation"]
        z_offset = meta["z_offset"]
        mesh_path = os.path.join(asset_root, model["category"], filename)
        
        world_xml += f"""
<model name="{model['name']}">
  <static>true</static>
  <pose>{x:.2f} {y:.2f} {z_offset:.2f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
  <link name="link">
    <!-- Collision meshes disabled for validation -->
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>file://{mesh_path}</uri>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
"""
        x += spacing

    lib = AssetLibrary()
    # Add 5-8 trees with large gaps as well
    for i in range(1, 6):
        world_xml += lib.spawn_tree(f"validation_tree_{i}", x, y, 0.0)
        x += spacing

    world_xml += """
</world>
</sdf>
"""
    return world_xml

if __name__ == "__main__":
    output_path = "/home/capstone/capstone_project_antigravity/world_generator/validation_world.sdf"
    with open(output_path, "w") as f:
        f.write(generate_validation_world())
    print(f"Validation world generated at {output_path}")
