from config import *

# ==========================================================
# Materials
# ==========================================================

ROAD_AMBIENT = "0.10 0.10 0.10 1"
ROAD_DIFFUSE = "0.10 0.10 0.10 1"

SIDEWALK_AMBIENT = "0.72 0.72 0.72 1"
SIDEWALK_DIFFUSE = "0.72 0.72 0.72 1"

LANE_AMBIENT = "0.95 0.95 0.70 1"
LANE_DIFFUSE = "0.95 0.95 0.70 1"


# ==========================================================
# Ground
# ==========================================================

def ground():

    ambient, diffuse = GROUND[GROUND_TYPE]

    return f"""
    <scene>

      <ambient>0.65 0.65 0.65 1</ambient>

      <background>0.72 0.84 0.97 1</background>

      <grid>false</grid>

      <origin_visual>false</origin_visual>

    </scene>

    <light name="sun" type="directional">

      <cast_shadows>true</cast_shadows>

      <pose>0 0 200 0 0 0</pose>

      <diffuse>1 1 1 1</diffuse>

      <specular>0.2 0.2 0.2 1</specular>

      <direction>-0.4 0.2 -1</direction>

    </light>

    <model name="ground_plane">

      <static>true</static>

      <link name="link">

        <collision name="collision">

          <geometry>

            <plane>

              <normal>0 0 1</normal>

              <size>1000 1000</size>

            </plane>

          </geometry>

        </collision>

        <visual name="visual">

          <geometry>

            <plane>

              <normal>0 0 1</normal>

              <size>1000 1000</size>

            </plane>

          </geometry>

          <material>

            <ambient>{ambient}</ambient>

            <diffuse>{diffuse}</diffuse>

          </material>

        </visual>

      </link>

    </model>
"""


# ==========================================================
# Generic Box Model
# ==========================================================

def box_model(
    name,
    x,
    y,
    z,
    sx,
    sy,
    sz,
    ambient,
    diffuse
):

    return f"""
    <model name="{name}">

      <static>true</static>

      <pose>{x:.2f} {y:.2f} {z:.3f} 0 0 0</pose>

      <link name="link">

        <visual name="visual">

          <geometry>

            <box>

              <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>

            </box>

          </geometry>

          <material>

            <ambient>{ambient}</ambient>

            <diffuse>{diffuse}</diffuse>

          </material>

        </visual>

      </link>

    </model>
"""


# ==========================================================
# Road Surface
# ==========================================================

def road_surface(
    name,
    x,
    y,
    sx,
    sy
):

    return box_model(
        name,
        x,
        y,
        0.01,
        sx,
        sy,
        0.02,
        ROAD_AMBIENT,
        ROAD_DIFFUSE
    )


# ==========================================================
# Sidewalk
# ==========================================================

def sidewalk(
    name,
    x,
    y,
    sx,
    sy
):

    return box_model(
        name,
        x,
        y,
        0.03,
        sx,
        sy,
        0.06,
        SIDEWALK_AMBIENT,
        SIDEWALK_DIFFUSE
    )


# ==========================================================
# Lane Marking
# ==========================================================

def lane_marking(
    name,
    x,
    y,
    sx,
    sy
):

    return box_model(
        name,
        x,
        y,
        0.041,
        sx,
        sy,
        0.005,
        LANE_AMBIENT,
        LANE_DIFFUSE
    )

# ==========================================================
# Road Network
# ==========================================================

def roads():

    xml = ""

    spacing = SPACING

    city_width = COLS * SPACING
    city_height = ROWS * SPACING
    
    # We use START_X_ROAD and START_Y_ROAD from config.py 
    # to center the roads in the authoritative world.
    start_x = START_X_ROAD + CITY_WIDTH/2
    start_y = START_Y_ROAD + CITY_HEIGHT/2

    sidewalk_width = max(1.5, SIDEWALK)
    lane_width = 0.30

    # ------------------------------------------------------
    # Horizontal Roads
    # ------------------------------------------------------

    for r in range(ROWS + 1):

        y = START_Y_ROAD + r * spacing

        xml += road_surface(
            f"hroad_{r}",
            start_x, # Center of the road X
            y,
            city_width + ROAD_WIDTH, # Full width including end roads
            ROAD_WIDTH
        )

        # Upper sidewalk
        xml += sidewalk(
            f"hside_top_{r}",
            start_x,
            y + ROAD_WIDTH / 2 + sidewalk_width / 2,
            city_width + ROAD_WIDTH,
            sidewalk_width
        )

        # Lower sidewalk
        xml += sidewalk(
            f"hside_bottom_{r}",
            start_x,
            y - ROAD_WIDTH / 2 - sidewalk_width / 2,
            city_width + ROAD_WIDTH,
            sidewalk_width
        )

        # Centre lane marking
        xml += lane_marking(
            f"hline_{r}",
            start_x,
            y,
            city_width + ROAD_WIDTH,
            lane_width
        )

    # ------------------------------------------------------
    # Vertical Roads
    # ------------------------------------------------------

    for c in range(COLS + 1):

        x = START_X_ROAD + c * spacing

        xml += road_surface(
            f"vroad_{c}",
            x,
            start_y,
            ROAD_WIDTH,
            city_height + ROAD_WIDTH
        )

        # Left sidewalk
        xml += sidewalk(
            f"vside_left_{c}",
            x - ROAD_WIDTH / 2 - sidewalk_width / 2,
            start_y,
            sidewalk_width,
            city_height + ROAD_WIDTH
        )

        # Right sidewalk
        xml += sidewalk(
            f"vside_right_{c}",
            x + ROAD_WIDTH / 2 + sidewalk_width / 2,
            start_y,
            sidewalk_width,
            city_height + ROAD_WIDTH
        )

        # Centre lane marking
        xml += lane_marking(
            f"vline_{c}",
            x,
            start_y,
            lane_width,
            city_height + ROAD_WIDTH
        )

    # ------------------------------------------------------
    # Intersections
    # ------------------------------------------------------

    for r in range(ROWS + 1):

        y = START_Y_ROAD + r * spacing

        for c in range(COLS + 1):

            x = START_X_ROAD + c * spacing

            xml += lane_marking(
                f"intersection_{r}_{c}",
                x,
                y,
                ROAD_WIDTH,
                lane_width
            )

            xml += lane_marking(
                f"intersection_cross_{r}_{c}",
                x,
                y,
                lane_width,
                ROAD_WIDTH
            )

    return xml