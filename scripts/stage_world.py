#!/usr/bin/env python3
"""
Staging World Generator for Forest Drone Simulation
Safely injects drone include block and required Ogre2 textures into a staged SDF world.
Performs XML validation and verifies pose coordinates.
"""

import sys
import os
import argparse
import xml.etree.ElementTree as ET
import numpy as np

import struct
import zlib

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


def write_minimal_png(filepath: str, width: int, height: int, r: int, g: int, b: int) -> None:
    """Generate a valid solid-color RGB PNG using only Python standard library."""
    raw_data = bytearray()
    row = bytes([0]) + bytes([r, g, b] * width)
    for _ in range(height):
        raw_data.extend(row)
    compressed = zlib.compress(bytes(raw_data), level=9)

    def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(chunk_type + data) & 0xffffffff
        return struct.pack('>I', len(data)) + chunk_type + data + struct.pack('>I', crc)

    header = b'\x89PNG\r\n\x1a\n'
    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    ihdr = png_chunk(b'IHDR', ihdr_data)
    idat = png_chunk(b'IDAT', compressed)
    iend = png_chunk(b'IEND', b'')

    with open(filepath, 'wb') as f:
        f.write(header + ihdr + idat + iend)


def main():
    parser = argparse.ArgumentParser(description="Stage a world with x500_thermal drone.")
    parser.add_argument("--base-world", required=True, help="Path to base SDF world")
    parser.add_argument("--output-world", required=True, help="Path to output staged SDF world")
    parser.add_argument("--drone-x", type=float, required=True, help="Drone X coordinate")
    parser.add_argument("--drone-y", type=float, required=True, help="Drone Y coordinate")
    parser.add_argument("--drone-z", type=float, required=True, help="Drone Z coordinate")
    parser.add_argument("--log-dir", required=True, help="Directory for textures and logs")
    args = parser.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)

    # 1. Ensure dummy terrain textures exist to satisfy Ogre2 Terra GLSL shader
    diff_path = os.path.join(args.log_dir, "terrain_diffuse.png")
    norm_path = os.path.join(args.log_dir, "terrain_normal.png")
    
    if not os.path.exists(diff_path) or os.path.getsize(diff_path) == 0:
        written = False
        if CV_AVAILABLE:
            try:
                cv2.imwrite(diff_path, np.full((16, 16, 3), (60, 100, 60), dtype=np.uint8))
                written = (os.path.exists(diff_path) and os.path.getsize(diff_path) > 0)
            except Exception:
                written = False
        if not written:
            write_minimal_png(diff_path, 16, 16, 60, 100, 60)

    if not os.path.exists(norm_path) or os.path.getsize(norm_path) == 0:
        written = False
        if CV_AVAILABLE:
            try:
                cv2.imwrite(norm_path, np.full((16, 16, 3), (128, 128, 255), dtype=np.uint8))
                written = (os.path.exists(norm_path) and os.path.getsize(norm_path) > 0)
            except Exception:
                written = False
        if not written:
            write_minimal_png(norm_path, 16, 16, 128, 128, 255)

    # Strict verification that textures exist and are non-empty
    if not (os.path.exists(diff_path) and os.path.getsize(diff_path) > 0):
        print(f"Error: Failed to create terrain diffuse texture at {diff_path}", file=sys.stderr)
        sys.exit(1)
    if not (os.path.exists(norm_path) and os.path.getsize(norm_path) > 0):
        print(f"Error: Failed to create terrain normal texture at {norm_path}", file=sys.stderr)
        sys.exit(1)

    with open(args.base_world, "r") as f:
        content = f.read()

    # 2. Add texture to heightmap if not present
    if "<texture>" not in content:
        tex_block = f"""<use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://{diff_path}</diffuse>
                <normal>file://{norm_path}</normal>
                <size>10</size>
              </texture>"""
        content = content.replace("<use_terrain_paging>false</use_terrain_paging>", tex_block, 1)

    # 3. Ensure spherical coordinates exist for Gazebo NavSat & Magnetometer sensors
    spherical_block = ""
    if "<spherical_coordinates>" not in content:
        spherical_block = """  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg>
    <elevation>0.0</elevation>
  </spherical_coordinates>
"""

    # 4. Format drone include block with explicit float substitution
    drone_pose_str = f"{args.drone_x:.6f} {args.drone_y:.6f} {args.drone_z:.6f} 0 0 0"
    drone_include = f"""{spherical_block}  <include>
    <name>x500_thermal_0</name>
    <uri>model://x500_thermal</uri>
    <pose>{drone_pose_str}</pose>
  </include>
</world>"""

    if "</world>" not in content:
        print("Error: </world> closing tag not found in base world!", file=sys.stderr)
        sys.exit(1)

    new_content = content.replace("</world>", drone_include, 1)

    # 4. XML validation check
    try:
        root = ET.fromstring(new_content)
    except ET.ParseError as e:
        print(f"Error: Generated SDF is not valid XML: {e}", file=sys.stderr)
        sys.exit(1)

    # Verify drone include element and pose
    found_drone = False
    for include in root.findall(".//include"):
        name_elem = include.find("name")
        pose_elem = include.find("pose")
        if name_elem is not None and name_elem.text == "x500_thermal_0":
            if pose_elem is not None and pose_elem.text.strip() == drone_pose_str:
                found_drone = True
                break

    if not found_drone:
        print("Error: Injected drone element verification failed!", file=sys.stderr)
        sys.exit(1)

    with open(args.output_world, "w") as f:
        f.write(new_content)

    print(f"Successfully staged world at: {args.output_world}")
    print(f"Verified x500_thermal_0 pose: {drone_pose_str}")
    sys.exit(0)


if __name__ == "__main__":
    main()
