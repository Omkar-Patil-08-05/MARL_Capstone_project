import json
import os
from generator import world

OUTPUT_SDF = os.path.expanduser(
    "~/capstone_project_antigravity/worlds/realistic_sar.sdf"
)
OUTPUT_META = os.path.expanduser(
    "~/capstone_project_antigravity/worlds/generated_world_meta.json"
)

def main():
    sdf, metadata = world()
    
    with open(OUTPUT_SDF, "w") as f:
        f.write(sdf)
        
    with open(OUTPUT_META, "w") as f:
        json.dump(metadata, f, indent=4)
        
    print()
    print("Generated")
    print(OUTPUT_SDF)
    print(OUTPUT_META)

if __name__ == "__main__":
    main()