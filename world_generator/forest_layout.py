import os
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
from forest_config import (
    WORLD_SIZE, HEIGHTMAP_RESOLUTION, MAX_HEIGHT, WATER_LEVEL, RNG,
    BASE_SIZE, BASE_CLEARING_RADIUS, SETTLEMENT_SIZE,
    VICTIM_GROUND_OFFSET, VICTIM_MAX_SLOPE_DEG,
    VICTIM_MIN_TREE_CLEARANCE, VICTIM_MIN_WATER_CLEARANCE_Z,
    VICTIM_MIN_INTER_DISTANCE, VICTIM_BOUNDARY_MARGIN
)

class ForestLayoutManager:
    def __init__(self, terrain_map, max_height=None):
        self.terrain = terrain_map
        self.max_height = max_height if max_height is not None else float(np.max(terrain_map))
        self.size = HEIGHTMAP_RESOLUTION
        
        self.pixel_size_m = WORLD_SIZE / (self.size - 1)
        elevations_m = self.terrain
        grad_y, grad_x = np.gradient(elevations_m, self.pixel_size_m)
        gradient_mag = np.sqrt(grad_x**2 + grad_y**2)
        self.slope_deg = np.degrees(np.arctan(gradient_mag))
        
        # Occupancy grid (0 = free for forest placement, 1 = excluded)
        self.occupancy = np.zeros((self.size, self.size), dtype=np.uint8)
        
        # Base Camp coordinates in Gazebo world: (-150.0, 150.0)
        self.base_world_x = -150.0
        self.base_world_y = 150.0
        self.settle_world_x = 120.0
        self.settle_world_y = -50.0
        
        self.base_center = self.get_image_coords(self.base_world_x, self.base_world_y)
        self.settlement_center = self.get_image_coords(self.settle_world_x, self.settle_world_y)
        
        self.zones = {
            "Base Camp (Sector 1)": (self.base_center[0], self.base_center[1], int((BASE_CLEARING_RADIUS / WORLD_SIZE) * self.size)),
            "Settlement Zone": (self.settlement_center[0], self.settlement_center[1], int((SETTLEMENT_SIZE / WORLD_SIZE) * self.size))
        }
        
        self.tree_records = []
        self.build_occupancy()
        
    def get_world_coords(self, col, row):
        """
        Convert heightmap image indices (col, row) to Gazebo world coordinates (x, y).
        Gazebo heightmap mapping:
        - col 0 maps to X = -WORLD_SIZE / 2; col N-1 maps to X = +WORLD_SIZE / 2 (West to East).
        - row 0 maps to Y = +WORLD_SIZE / 2; row N-1 maps to Y = -WORLD_SIZE / 2 (North to South).
        """
        x = (col / (self.size - 1)) * WORLD_SIZE - (WORLD_SIZE / 2.0)
        y = (WORLD_SIZE / 2.0) - (row / (self.size - 1)) * WORLD_SIZE
        return float(x), float(y)
        
    def get_image_coords(self, x, y):
        """
        Convert Gazebo world coords (x, y) to heightmap image indices (col, row).
        """
        col = ((x + WORLD_SIZE / 2.0) / WORLD_SIZE) * (self.size - 1)
        row = ((WORLD_SIZE / 2.0 - y) / WORLD_SIZE) * (self.size - 1)
        return int(np.clip(round(col), 0, self.size - 1)), int(np.clip(round(row), 0, self.size - 1))
        
    def get_height(self, x, y):
        col, row = self.get_image_coords(x, y)
        return float(self.terrain[row, col])

    def get_slope_deg(self, x, y):
        col, row = self.get_image_coords(x, y)
        return float(self.slope_deg[row, col])

    def build_occupancy(self):
        """Mark exclusion zones where trees must never spawn."""
        # 1. Water channel exclusion (Z <= WATER_LEVEL + 0.6m)
        water_thresh = WATER_LEVEL + 0.6
        self.occupancy[self.terrain <= water_thresh] = 1
        
        # 2. Extreme cliff exclusion (> 32 degrees)
        self.occupancy[self.slope_deg > 32.0] = 1
        
        # 3. Base Camp clearing (32m radius around launch pads)
        bc_col, bc_row = self.base_center
        bc_radius_px = int((BASE_CLEARING_RADIUS / WORLD_SIZE) * self.size)
        cv2.circle(self.occupancy, (bc_col, bc_row), bc_radius_px, 1, -1)
        
        # 4. Settlement clearing
        sc_col, sc_row = self.settlement_center
        sc_radius_px = int((SETTLEMENT_SIZE / 2.0 / WORLD_SIZE) * self.size)
        cv2.circle(self.occupancy, (sc_col, sc_row), sc_radius_px, 1, -1)

    def generate_clustered_trees(self, total_count=900):
        """
        Generates dense, natural stands of authentic 3D trees:
        - Dense woodland across substantial portions of the terrain
        - Clustered groves with overlapping canopies (15m to 40m radii)
        - Background continuous woodland fill connecting clusters
        - Concentrated density in the SAR search sector and forested slopes
        - Clearings for staging base, riverbanks, and settlement
        """
        trees = []
        free_rows, free_cols = np.where(self.occupancy == 0)
        num_free = len(free_rows)
        if num_free == 0:
            return trees

        # Generate 55 cluster centers
        cluster_centers = []
        
        # Operational search bounds: [-180, 20] x [-20, 180]
        # 35 clusters distributed across operational search area ridges and valleys
        for _ in range(35):
            cx = RNG.uniform(-170.0, 15.0)
            cy = RNG.uniform(-15.0, 175.0)
            if math.hypot(cx - self.base_world_x, cy - self.base_world_y) > BASE_CLEARING_RADIUS + 6.0:
                cluster_centers.append((cx, cy))
                
        # 20 clusters in surrounding hills, valleys, and outer ridges
        for _ in range(20):
            idx = RNG.randint(0, num_free - 1)
            cx, cy = self.get_world_coords(free_cols[idx], free_rows[idx])
            cluster_centers.append((cx, cy))

        attempts = 0
        max_attempts = total_count * 30

        while len(trees) < total_count and attempts < max_attempts:
            attempts += 1
            if RNG.random() < 0.72:
                # 72% placed in natural overlapping groves
                center_x, center_y = RNG.choice(cluster_centers)
                radius = abs(RNG.gauss(0.0, RNG.uniform(16.0, 32.0)))
                angle = RNG.uniform(0.0, 2.0 * math.pi)
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
            else:
                # 28% natural woodland fill connecting stands across terrain
                idx = RNG.randint(0, num_free - 1)
                x, y = self.get_world_coords(free_cols[idx], free_rows[idx])
                x += RNG.uniform(-6.0, 6.0)
                y += RNG.uniform(-6.0, 6.0)

            if abs(x) > (WORLD_SIZE / 2.0 - 15.0) or abs(y) > (WORLD_SIZE / 2.0 - 15.0):
                continue

            dist_to_base = math.hypot(x - self.base_world_x, y - self.base_world_y)
            if dist_to_base < BASE_CLEARING_RADIUS:
                continue

            col, row = self.get_image_coords(x, y)
            if self.occupancy[row, col] != 0:
                continue

            z = self.get_height(x, y)
            if z <= WATER_LEVEL + 0.6:
                continue

            # Species distribution
            species_roll = RNG.random()
            if species_roll < 0.48:
                species = "pine_tree" # Open Robotics photorealistic Pine
                scale = RNG.uniform(1.4, 2.3)
                canopy_r = 2.4 * (scale / 1.5)
                trunk_r = 0.38 * (scale / 1.5)
            elif species_roll < 0.82:
                species = "oak_tree" # Open Robotics photorealistic Oak
                scale = RNG.uniform(1.1, 1.6)
                canopy_r = 4.0 * (scale / 1.2)
                trunk_r = 0.45 * (scale / 1.2)
            elif species_roll < 0.93:
                species = "pine_tall" # Kenney Conifer
                scale = RNG.uniform(6.0, 8.0)
                canopy_r = 2.2 * (scale / 6.5)
                trunk_r = 0.35 * (scale / 6.5)
            else:
                species = "broadleaf_dense" # Kenney Broadleaf
                scale = RNG.uniform(5.5, 7.5)
                canopy_r = 3.2 * (scale / 6.0)
                trunk_r = 0.40 * (scale / 6.0)

            yaw = RNG.uniform(0.0, 2.0 * math.pi)

            tree_entry = {
                "species": species,
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw": float(yaw),
                "scale": float(scale),
                "canopy_radius": float(canopy_r),
                "trunk_radius": float(trunk_r)
            }
            trees.append(tree_entry)
            self.tree_records.append(tree_entry)

        return trees

    def generate_understory_bushes(self, count=220):
        bushes = []
        if not self.tree_records:
            return bushes

        for _ in range(count):
            parent_tree = RNG.choice(self.tree_records)
            offset_dist = RNG.uniform(2.2, 7.5)
            offset_angle = RNG.uniform(0.0, 2.0 * math.pi)
            x = parent_tree["x"] + offset_dist * math.cos(offset_angle)
            y = parent_tree["y"] + offset_dist * math.sin(offset_angle)

            if abs(x) > (WORLD_SIZE / 2.0 - 20.0) or abs(y) > (WORLD_SIZE / 2.0 - 20.0):
                continue

            dist_to_base = math.hypot(x - self.base_world_x, y - self.base_world_y)
            if dist_to_base < BASE_CLEARING_RADIUS - 4.0:
                continue

            z = self.get_height(x, y)
            if z <= WATER_LEVEL + 0.4:
                continue

            scale = RNG.uniform(3.5, 5.5)
            yaw = RNG.uniform(0.0, 2.0 * math.pi)
            bush_type = "plant_bush" if RNG.random() < 0.6 else "plant_bush_detailed"

            bushes.append({
                "model": bush_type,
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw": float(yaw),
                "scale": float(scale)
            })
        return bushes

    def generate_fallen_logs(self, count=40):
        logs = []
        free_rows, free_cols = np.where(self.occupancy == 0)
        num_free = len(free_rows)
        if num_free == 0:
            return logs

        for _ in range(count):
            idx = RNG.randint(0, num_free - 1)
            x, y = self.get_world_coords(free_cols[idx], free_rows[idx])
            z = self.get_height(x, y)
            if z <= WATER_LEVEL + 0.8:
                continue

            dist_to_base = math.hypot(x - self.base_world_x, y - self.base_world_y)
            if dist_to_base < BASE_CLEARING_RADIUS:
                continue

            log_type = "stump_old" if RNG.random() < 0.6 else "stump_tall"
            scale = RNG.uniform(3.0, 4.5)
            yaw = RNG.uniform(0.0, 2.0 * math.pi)

            logs.append({
                "model": log_type,
                "x": float(x),
                "y": float(y),
                "z": float(z + 0.08), # Natural terrain grounding
                "yaw": float(yaw),
                "scale": float(scale)
            })
        return logs

    def generate_riverbank_rocks(self, count=45):
        rocks = []
        lin = np.linspace(-WORLD_SIZE / 2.0 + 30.0, WORLD_SIZE / 2.0 - 30.0, 250)
        for _ in range(count):
            x = float(RNG.choice(lin))
            y = float(RNG.choice(lin))
            z = self.get_height(x, y)
            # Riverbank: elevation between WATER_LEVEL + 0.3m and WATER_LEVEL + 2.5m
            is_bank = (WATER_LEVEL + 0.2 <= z <= WATER_LEVEL + 2.8)
            is_ridge = (z >= 36.0 and RNG.random() < 0.3)
            if is_bank or is_ridge:
                scale = RNG.uniform(3.5, 6.0)
                yaw = RNG.uniform(0.0, 2.0 * math.pi)
                rocks.append({
                    "model": "stone_large",
                    "x": float(x),
                    "y": float(y),
                    "z": float(z - 0.25), # Partially embedded in riverbed / earth
                    "yaw": float(yaw),
                    "scale": float(scale)
                })
        return rocks

    def generate_victims(self, count=5):
        victims = []
        free_rows, free_cols = np.where(self.occupancy == 0)
        num_free = len(free_rows)
        if num_free == 0:
            return victims

        victim_models = [
            ("casual_female", 0.02),
            ("rescue_randy_standing", 0.02),
            ("rescue_randy_sitting", 0.02)
        ]

        # Operational SAR search grid bounds: [-180, 20] x [-20, 180]
        # Keep 10m margin from operational outer edges
        SEARCH_MIN_X = -170.0
        SEARCH_MAX_X = 10.0
        SEARCH_MIN_Y = -10.0
        SEARCH_MAX_Y = 170.0

        target_sectors = [
            ("Sector 3 (Northwest Ridge)", -85.0, 120.0, 45.0),
            ("Sector 2 (River Corridor)", -40.0, 60.0, 55.0),
            ("Sector 4 (East Woodland)", -5.0, 75.0, 45.0),
            ("Sector 1 (Base Staging)", -150.0, 150.0, 35.0),
        ]

        attempts = 0
        max_attempts = count * 600

        while len(victims) < count and attempts < max_attempts:
            attempts += 1
            idx = RNG.randint(0, num_free - 1)
            x, y = self.get_world_coords(free_cols[idx], free_rows[idx])

            # Strictly inside active 200m x 200m operational search area
            if not (SEARCH_MIN_X <= x <= SEARCH_MAX_X and SEARCH_MIN_Y <= y <= SEARCH_MAX_Y):
                continue

            dist_to_base = math.hypot(x - self.base_world_x, y - self.base_world_y)
            if dist_to_base < BASE_CLEARING_RADIUS + 10.0:
                continue

            z = self.get_height(x, y)
            if z <= WATER_LEVEL + VICTIM_MIN_WATER_CLEARANCE_Z:
                continue

            slope = self.get_slope_deg(x, y)
            if slope > VICTIM_MAX_SLOPE_DEG + 4.0:
                continue

            too_close_to_tree = False
            for t in self.tree_records:
                dist_t = math.hypot(x - t["x"], y - t["y"])
                if dist_t < (t["trunk_radius"] + VICTIM_MIN_TREE_CLEARANCE):
                    too_close_to_tree = True
                    break
            if too_close_to_tree:
                continue

            too_close_to_victim = False
            for v in victims:
                dist_v = math.hypot(x - v["x"], y - v["y"])
                if dist_v < VICTIM_MIN_INTER_DISTANCE:
                    too_close_to_victim = True
                    break
            if too_close_to_victim:
                continue

            # Prioritize distinct sector assignment for variety
            closest_sector = min(target_sectors, key=lambda s: math.hypot(x - s[1], y - s[2]))[0]

            v_type, ground_offset = victim_models[len(victims) % len(victim_models)]
            yaw = RNG.uniform(0.0, 2.0 * math.pi)

            victims.append({
                "id": len(victims),
                "model": v_type,
                "x": float(x),
                "y": float(y),
                "z": float(z + ground_offset),
                "yaw": float(yaw),
                "ground_z": float(z),
                "slope_deg": float(slope),
                "sector": closest_sector
            })

        return victims

    def render_2d_map(self, trees, victims, out_path):
        map_size = 800
        # self.terrain: row 0 is North (+250), col 0 is West (-250) -> naturally North-up, East-right
        elev_m = cv2.resize(self.terrain, (map_size, map_size), interpolation=cv2.INTER_LINEAR)
        
        map_rgb = np.zeros((map_size, map_size, 3), dtype=np.uint8)
        water_thresh = WATER_LEVEL + 0.3
        
        for r in range(map_size):
            for c in range(map_size):
                h = elev_m[r, c]
                if h <= water_thresh:
                    map_rgb[r, c] = [38, 102, 155] # River water
                elif h < 16.0:
                    factor = np.clip((h - water_thresh) / (16.0 - water_thresh), 0.0, 1.0)
                    map_rgb[r, c] = [int(32 + 15 * factor), int(78 + 25 * factor), int(42 + 10 * factor)]
                elif h < 32.0:
                    factor = np.clip((h - 16.0) / 16.0, 0.0, 1.0)
                    map_rgb[r, c] = [int(48 - 18 * factor), int(105 - 25 * factor), int(52 - 18 * factor)]
                else:
                    map_rgb[r, c] = [65, 80, 58]

        # Draw dense tree canopies (East-right, North-up)
        for t in trees:
            col = int(((t["x"] + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
            row = int(((-t["y"] + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
            col = np.clip(col, 0, map_size - 1)
            row = np.clip(row, 0, map_size - 1)
            radius_px = max(3, int((t["canopy_radius"] / WORLD_SIZE) * map_size))
            if "pine" in t["species"]:
                canopy_color = (22, 58, 28)
            else:
                canopy_color = (36, 110, 42)
            cv2.circle(map_rgb, (col, row), radius_px, canopy_color, -1)

        # Base Camp apron
        bc_col = int(((self.base_world_x + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
        bc_row = int(((-self.base_world_y + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
        bc_r_px = int((BASE_CLEARING_RADIUS / WORLD_SIZE) * map_size)
        cv2.circle(map_rgb, (bc_col, bc_row), bc_r_px, (95, 105, 90), -1)
        cv2.circle(map_rgb, (bc_col, bc_row), bc_r_px, (145, 155, 135), 2)
        cv2.putText(map_rgb, "SAR BASE CAMP", (bc_col - 55, bc_row + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 245, 245), 1)

        # Operational Search Area Box (200m x 200m)
        s_min_c = int(((-180.0 + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
        s_max_c = int(((20.0 + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
        s_min_r = int(((-180.0 + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
        s_max_r = int(((20.0 + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
        cv2.rectangle(map_rgb, (s_min_c, s_min_r), (s_max_c, s_max_r), (210, 210, 110), 2)

        # Victims
        for v in victims:
            col = int(((v["x"] + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
            row = int(((-v["y"] + WORLD_SIZE / 2.0) / WORLD_SIZE) * map_size)
            col = np.clip(col, 0, map_size - 1)
            row = np.clip(row, 0, map_size - 1)
            cv2.circle(map_rgb, (col, row), 5, (235, 85, 45), -1)
            cv2.circle(map_rgb, (col, row), 7, (255, 255, 255), 1)

        map_bgr = cv2.cvtColor(map_rgb, cv2.COLOR_RGB2BGR)
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        cv2.imwrite(out_path, map_bgr)
        return out_path
