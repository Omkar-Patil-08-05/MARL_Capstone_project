import os
import random

# ==========================================================
# Asset Root
# ==========================================================

ASSET_ROOT = os.path.expanduser(
    "~/capstone_project_antigravity/assets_real"
)

# ==========================================================
# Category Configuration
# ==========================================================

ASSET_METADATA = {
    "house.glb": {"z_offset": -42.50, "rotation": (1.5708, 0.0, 0.0), "size_x": 21.89, "size_y": 21.89, "category": "Houses"},
    "small_house.glb": {"z_offset": 0.00, "rotation": (1.5708, 0.0, 0.0), "size_x": 3.73, "size_y": 6.00, "category": "Houses"},
    "abandoned_building.glb": {"z_offset": -0.03, "rotation": (1.5708, 0.0, 0.0), "size_x": 20.54, "size_y": 15.00, "category": "Buildings"},
    "abandoned_construction_building.glb": {"z_offset": 0.00, "rotation": (1.5708, 0.0, 0.0), "size_x": 15.76, "size_y": 15.00, "category": "Buildings"},
    "futuristic_building.glb": {"z_offset": -5.10, "rotation": (1.5708, 0.0, 0.0), "size_x": 21.89, "size_y": 15.00, "category": "Buildings"},
    "low_poly_-_soviet__apartment_building_8k.glb": {"z_offset": 0.00, "rotation": (1.5708, 0.0, 0.0), "size_x": 8.92, "size_y": 21.70, "category": "Buildings"},
    "post-apocalyptic_buildings.glb": {"z_offset": -5.81, "rotation": (1.5708, 0.0, 0.0), "size_x": 1.99, "size_y": 15.00, "category": "Buildings"},
    "residential_complex_modern_apartment_building.glb": {"z_offset": 0.30, "rotation": (1.5708, 0.0, 0.0), "size_x": 25.70, "size_y": 15.00, "category": "Buildings"},
    "old_rusty_car.glb": {"z_offset": 4.69, "rotation": (1.5708, 0.0, 0.0), "size_x": 4.50, "size_y": 5.65, "category": "Cars"},
    "small_price_car.glb": {"z_offset": 2.31, "rotation": (1.5708, 0.0, 0.0), "size_x": 4.50, "size_y": 10.16, "category": "Cars"},
    "brick_rubble_scaniverse_lidar.glb": {"z_offset": 0.00, "rotation": (1.5708, 0.0, 0.0), "size_x": 16.85, "size_y": 4.39, "category": "Rubble"},
    "construction_rubble.glb": {"z_offset": 0.00, "rotation": (1.5708, 0.0, 0.0), "size_x": 2.65, "size_y": 1.50, "category": "Rubble"},
    "40_meter_radiotower.glb": {"z_offset": 6.71, "rotation": (1.5708, 0.0, 0.0), "size_x": 4.33, "size_y": 4.26, "category": "Towers"}
}

CATEGORIES = ["Buildings", "Houses", "Cars", "Rubble", "Towers"]

# ==========================================================
# Asset Library
# ==========================================================


class AssetLibrary:

    def __init__(self):

        self.assets = {}

        self.scan()

    # ------------------------------------------------------

    def scan(self):

        self.assets.clear()

        for category in CATEGORIES:

            folder = os.path.join(
                ASSET_ROOT,
                category
            )

            models = []

            if os.path.isdir(folder):

                for file in sorted(os.listdir(folder)):

                    if file.lower().endswith(".glb"):

                        models.append(
                            os.path.join(folder, file)
                        )

            self.assets[category] = models

    # ------------------------------------------------------

    def random_asset(self, category):

        models = self.assets.get(category, [])

        if not models:

            raise RuntimeError(
                f"No GLB models found in:\n"
                f"{os.path.join(ASSET_ROOT, category)}"
            )

        return random.choice(models)
    
    def random_asset_fitting(self, category, max_size_x, max_size_y, rng=random):
        models = self.assets.get(category, [])
        valid = []
        for path in models:
            filename = os.path.basename(path)
            if filename in ASSET_METADATA:
                meta = ASSET_METADATA[filename]
                # Check both orientations
                fits_normal = meta["size_x"] <= max_size_x and meta["size_y"] <= max_size_y
                fits_rotated = meta["size_y"] <= max_size_x and meta["size_x"] <= max_size_y
                if fits_normal or fits_rotated:
                    valid.append(path)
        if not valid:
            return None
        return rng.choice(valid)

    # ------------------------------------------------------

    def rotation(self, filename):
        return ASSET_METADATA.get(filename, {}).get("rotation", (0.0, 0.0, 0.0))

    # ------------------------------------------------------

    def z_offset(self, filename):
        return ASSET_METADATA.get(filename, {}).get("z_offset", 0.0)

    # ------------------------------------------------------

    def collision_enabled(self, filename):
        return False

    # ------------------------------------------------------

    def mesh_visual(self, mesh):

        return f"""
      <visual name="visual">

        <geometry>

          <mesh>

            <uri>file://{mesh}</uri>

          </mesh>

        </geometry>

      </visual>
"""

    # ------------------------------------------------------

    def mesh_collision(self, mesh):

        return f"""
      <collision name="collision">

        <geometry>

          <mesh>

            <uri>file://{mesh}</uri>

          </mesh>

        </geometry>

      </collision>
"""

    # ------------------------------------------------------

    def model_xml(
        self,
        name,
        category,
        x,
        y,
        z,
        yaw=0.0
    ):

        mesh = self.random_asset(category)

        filename = os.path.basename(mesh)

        roll, pitch, _ = self.rotation(filename)

        z += self.z_offset(filename)

        collision = ""

        if self.collision_enabled(filename):

            collision = self.mesh_collision(mesh)

        return f"""
  <model name="{name}">

    <static>true</static>

    <pose>
      {x:.2f}
      {y:.2f}
      {z:.2f}
      {roll:.6f}
      {pitch:.6f}
      {yaw:.6f}
    </pose>

    <link name="link">

{collision}

{self.mesh_visual(mesh)}

    </link>

  </model>
"""

    def spawn_tree(self, name, x, y, z=0.0):
        return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
    <link name="link">
      <visual name="trunk">
        <pose>0 0 1.25 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>2.5</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.45 0.25 0.1 1</ambient>
          <diffuse>0.45 0.25 0.1 1</diffuse>
        </material>
      </visual>
      <!-- Leaves -->
      <visual name="leaves1">
        <pose>0 0 2.75 0 0 0</pose>
        <geometry><sphere><radius>0.8</radius></sphere></geometry>
        <material><ambient>0.0 0.6 0.0 1</ambient><diffuse>0.0 0.6 0.0 1</diffuse></material>
      </visual>
      <visual name="leaves2">
        <pose>0.3 0 2.95 0 0 0</pose>
        <geometry><sphere><radius>0.6</radius></sphere></geometry>
        <material><ambient>0.0 0.7 0.0 1</ambient><diffuse>0.0 0.7 0.0 1</diffuse></material>
      </visual>
      <visual name="leaves3">
        <pose>-0.3 0.2 2.85 0 0 0</pose>
        <geometry><sphere><radius>0.6</radius></sphere></geometry>
        <material><ambient>0.0 0.5 0.0 1</ambient><diffuse>0.0 0.5 0.0 1</diffuse></material>
      </visual>
    </link>
  </model>
"""

# ==========================================================
# Debug Utility
# ==========================================================

if __name__ == "__main__":

    library = AssetLibrary()

    print()

    print("Detected Assets")

    print("----------------------------")

    total = 0

    for category in library.assets:

        count = len(library.assets[category])

        total += count

        print(f"{category:12s}: {count}")

    print("----------------------------")

    print(f"Total Models : {total}")