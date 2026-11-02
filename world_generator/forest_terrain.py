import os
import numpy as np
import cv2
import math
from forest_config import WORLD_SIZE, HEIGHTMAP_RESOLUTION, MAX_HEIGHT, WATER_LEVEL

def generate_fractal_noise(size, seed, octaves=7, persistence=0.5, scale=1.0):
    """Generate multi-octave fractal value noise using smooth bicubic interpolation."""
    rng = np.random.default_rng(seed)
    noise = np.zeros((size, size), dtype=np.float32)
    amplitude = 1.0
    total_amplitude = 0.0
    
    for i in range(octaves):
        freq = 2 ** i
        grid_size = max(2, int(freq * scale))
        rand_grid = rng.random((grid_size, grid_size), dtype=np.float32)
        layer = cv2.resize(rand_grid, (size, size), interpolation=cv2.INTER_CUBIC)
        noise += layer * amplitude
        total_amplitude += amplitude
        amplitude *= persistence
        
    return noise / total_amplitude

def generate_forest_heightmap(seed, out_path):
    """
    Generates a realistic uneven terrain heightmap with:
    - Base landscape elevation strictly above water level (Z >= 6.5m everywhere).
    - Mountain ridges in the North and East (Z ~ 35m to 65m).
    - Rolling forested hills and valleys in the SAR search sector (Z ~ 10m to 28m).
    - A narrow, winding carved river gorge (width ~18m, riverbed Z ~ 1.5m).
    - Water level is at Z = 3.5m, so water ONLY exists inside the winding riverbed!
    - Flat elevated plateau for the drone staging base camp at (-150, 150) (Z = 14.0m).
    """
    size = HEIGHTMAP_RESOLUTION
    
    # Coordinates in world meters:
    # col 0 is X = -250.0 (West), col 512 is X = +250.0 (East)
    # row 0 is Y = +250.0 (North), row 512 is Y = -250.0 (South)
    lin_x = np.linspace(-250.0, 250.0, size, dtype=np.float32)
    lin_y = np.linspace(250.0, -250.0, size, dtype=np.float32)
    X, Y = np.meshgrid(lin_x, lin_y)
    
    # 1. Base rolling hills: 9.0m to 22.0m elevation
    hills_noise = generate_fractal_noise(size, seed, octaves=6, persistence=0.48, scale=3.5)
    base_terrain = 9.0 + hills_noise * 13.0
    
    # 2. Mountain Ridge (prominent in East and South-East, where X > 40m, well away from base camp)
    ridge_axis = ((X - 20.0) / 200.0) + 0.35 * ((Y + 40.0) / 250.0)
    ridge_mask = np.clip((ridge_axis - 0.05) / 0.75, 0.0, 1.0) ** 1.8
    ridge_noise = generate_fractal_noise(size, seed + 101, octaves=7, persistence=0.52, scale=5.5)
    mountain_elevation = ridge_mask * (10.0 + 26.0 * ridge_noise)
    terrain = base_terrain + mountain_elevation
    
    # 3. Winding River Corridor (narrow, natural meandering stream cutting North-South through valley):
    # Centerline in world coordinates (X meanders between -35m and +25m):
    river_center_x = 35.0 * np.sin(Y * (2.2 * np.pi / 250.0)) - 0.10 * Y + 10.0 * np.cos(Y * (4.5 * np.pi / 250.0))
    river_dist = np.abs(X - river_center_x)
    
    # The river channel has a narrow bed: width 17m (half-width 8.5m)
    # The river banks rise smoothly over 16m up to the rolling hills
    # Riverbed elevation is 1.75m, below water level 3.5m!
    channel_half_width = 8.5   # 17m water width
    bank_width = 16.0          # 16m bank slope
    
    river_t = np.clip((river_dist - channel_half_width) / bank_width, 0.0, 1.0)
    river_t = np.sin(river_t * (np.pi / 2.0)) ** 2.0 # Smooth S-curve transition
    
    river_bed = 1.75
    terrain = river_bed + (terrain - river_bed) * river_t
    
    # 4. Drone Staging Base Camp Plateau:
    # Located at Gazebo world (-150, 150), flat plateau at Z = 12.0m
    base_dist = np.sqrt((X - (-150.0))**2 + (Y - 150.0)**2)
    base_blend = np.clip((base_dist - 34.0) / 18.0, 0.0, 1.0)
    terrain = terrain * base_blend + 12.0 * (1.0 - base_blend)
    
    # 5. Settlement Clearing (East bank of river at Gazebo world (120, -50), Z ~ 15m)
    settle_dist = np.sqrt((X - 120.0)**2 + (Y - (-50.0))**2)
    settle_blend = np.clip((settle_dist - 28.0) / 16.0, 0.0, 1.0)
    terrain = terrain * settle_blend + 15.0 * (1.0 - settle_blend)
    
    # Pin corner [0, 0] to 0.0 to guarantee zero-datum for Gazebo ODE heightfield
    terrain[0, 0] = 0.0
    
    # Smooth slight artifacts with a small Gaussian blur
    terrain = cv2.GaussianBlur(terrain, (3, 3), 0.5)
    # Keep flat core exact 12.0m after blur
    terrain[base_dist <= 30.0] = 12.0
    terrain[settle_dist <= 24.0] = 15.0
    
    # Save as standard 16-bit PNG for Gazebo heightmap
    max_h = float(np.max(terrain))
    terrain_16 = np.clip((terrain / max_h) * 65535.0, 0, 65535).astype(np.uint16)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    cv2.imwrite(out_path, terrain_16)
    
    return terrain, max_h

def ensure_terrain_textures(out_dir):
    """
    Generates rich 512x512 organic forest floor diffuse and normal textures
    compatible with the OGRE 2 Terra heightmap shader.
    """
    os.makedirs(out_dir, exist_ok=True)
    diffuse_path = os.path.join(out_dir, "terrain_diffuse.png")
    normal_path = os.path.join(out_dir, "terrain_normal.png")
    
    size = 512
    rng = np.random.default_rng(4321)
    
    # Rich organic forest floor: deep mossy green, dark soil/loam, dry pine needle patches
    diff_img = np.zeros((size, size, 3), dtype=np.float32)
    diff_img[:, :] = [36, 70, 42]
    
    soil_noise = cv2.resize(rng.random((16, 16), dtype=np.float32), (size, size), interpolation=cv2.INTER_CUBIC)
    moss_noise = cv2.resize(rng.random((32, 32), dtype=np.float32), (size, size), interpolation=cv2.INTER_CUBIC)
    fine_noise = cv2.resize(rng.random((128, 128), dtype=np.float32), (size, size), interpolation=cv2.INTER_CUBIC)
    
    diff_img[:, :, 0] += (soil_noise - 0.5) * 18.0
    diff_img[:, :, 1] += (moss_noise - 0.4) * 32.0 + (fine_noise - 0.5) * 12.0
    diff_img[:, :, 2] += (soil_noise - 0.5) * 16.0
    
    diff_img = np.clip(diff_img, 0, 255).astype(np.uint8)
    cv2.imwrite(diffuse_path, diff_img)
    
    height_bump = (moss_noise * 0.5 + fine_noise * 0.5)
    gx = cv2.Sobel(height_bump, cv2.CV_32F, 1, 0, ksize=3) * 1.5
    gy = cv2.Sobel(height_bump, cv2.CV_32F, 0, 1, ksize=3) * 1.5
    
    norm_x = -gx
    norm_y = -gy
    norm_z = np.ones_like(gx)
    mag = np.sqrt(norm_x**2 + norm_y**2 + norm_z**2)
    norm_x /= mag
    norm_y /= mag
    norm_z /= mag
    
    b = np.clip((norm_x * 0.5 + 0.5) * 255, 0, 255).astype(np.uint8)
    g = np.clip((norm_y * 0.5 + 0.5) * 255, 0, 255).astype(np.uint8)
    r = np.clip((norm_z * 0.5 + 0.5) * 255, 0, 255).astype(np.uint8)
    
    normal_img = cv2.merge([b, g, r])
    cv2.imwrite(normal_path, normal_img)
    
    return diffuse_path, normal_path

def get_terrain_sdf(heightmap_path, diffuse_path=None, normal_path=None, max_height=MAX_HEIGHT):
    """
    Returns self-contained SDF XML for the generated heightmap and water plane.
    """
    texture_xml = ""
    if diffuse_path and normal_path and os.path.exists(diffuse_path) and os.path.exists(normal_path):
        texture_xml = f"""
              <texture>
                <diffuse>file://{diffuse_path}</diffuse>
                <normal>file://{normal_path}</normal>
                <size>12</size>
              </texture>"""

    return f"""
    <model name="forest_terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{heightmap_path}</uri>
              <size>{WORLD_SIZE} {WORLD_SIZE} {max_height:.4f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>{texture_xml}
              <uri>file://{heightmap_path}</uri>
              <size>{WORLD_SIZE} {WORLD_SIZE} {max_height:.4f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.24 0.35 0.20 1.0</ambient>
            <diffuse>0.28 0.40 0.22 1.0</diffuse>
            <specular>0.04 0.04 0.04 1.0</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Winding River Water Surface (confined to the natural river corridor) -->
    <model name="river_water">
      <static>true</static>
      <pose>-5.0 0 3.5 0 0 0</pose>
      <link name="link">
        <visual name="water_visual">
          <geometry>
            <box>
              <size>130.0 500.0 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.06 0.22 0.32 1.0</ambient>
            <diffuse>0.08 0.26 0.38 1.0</diffuse>
            <specular>0.50 0.60 0.70 1.0</specular>
          </material>
        </visual>
      </link>
    </model>
    """
