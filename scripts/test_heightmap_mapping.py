import os
import cv2
import numpy as np
import subprocess

size = 513
img = np.zeros((size, size), dtype=np.uint16)

# Put a 50m high plateau (50/70 * 65535 ~ 46810) ONLY in Top-Left (row 0..100, col 0..100)
img[0:120, 0:120] = 46810

# Put a 25m high plateau (25/70 * 65535 ~ 23405) in Bottom-Right (row 400..512, col 400..512)
img[400:512, 400:512] = 23405

test_png = "/tmp/test_corner_heightmap.png"
cv2.imwrite(test_png, img)

sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="test_coords">
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.5 0.5 0.5 1</background>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.2 0.2 -1</direction>
    </light>
    
    <model name="test_terrain">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <uri>file://{test_png}</uri>
              <size>500 500 70</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Red sphere at (+180, +180, 55): North-East -->
    <model name="sphere_ne">
      <static>true</static>
      <pose>180 180 55 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><sphere><radius>10</radius></sphere></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Green sphere at (-180, +180, 55): North-West -->
    <model name="sphere_nw">
      <static>true</static>
      <pose>-180 180 55 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><sphere><radius>10</radius></sphere></geometry>
          <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Blue sphere at (-180, -180, 55): South-West -->
    <model name="sphere_sw">
      <static>true</static>
      <pose>-180 -180 55 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><sphere><radius>10</radius></sphere></geometry>
          <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Yellow sphere at (+180, -180, 55): South-East -->
    <model name="sphere_se">
      <static>true</static>
      <pose>180 -180 55 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><sphere><radius>10</radius></sphere></geometry>
          <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Overhead camera looking straight down from Z=500 -->
    <!-- Yaw 0 means camera X is world X (East), camera Y is world Y (North) -->
    <!-- Pitch 1.57 (90 deg down) -->
    <model name="top_cam">
      <static>true</static>
      <pose>0 0 450 0 1.570796 0</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <camera>
            <horizontal_fov>1.4</horizontal_fov>
            <image><width>800</width><height>800</height></image>
            <clip><near>1</near><far>1000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>/top_cam/image</topic>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""

with open("/tmp/test_coords.sdf", "w") as f:
    f.write(sdf_content)

print("Test world created at /tmp/test_coords.sdf")
