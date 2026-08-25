import os
import config

# Patch config for small preview
config.ROWS = 3
config.COLS = 3
config.WORLD_SIZE = 120

from generator import world

OUTPUT = os.path.expanduser(
    "~/capstone_project_antigravity/worlds/preview_world.sdf"
)

with open(OUTPUT, "w") as f:
    f.write(world())

print()
print("Generated Preview")
print(OUTPUT)
