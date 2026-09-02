import json
import os
import argparse
from generator import world
from city import set_victim_config

OUTPUT_SDF = os.path.expanduser(
    "~/capstone_project_antigravity/worlds/realistic_sar.sdf"
)
OUTPUT_META = os.path.expanduser(
    "~/capstone_project_antigravity/worlds/generated_world_meta.json"
)

def main():
    parser = argparse.ArgumentParser(description="Generate SAR world SDF and metadata.")
    parser.add_argument("--victims", type=int, default=5,
                        help="Number of victims to place (1-10). Default: 5")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for victim placement. Default: 42")
    args = parser.parse_args()

    if args.victims < 1 or args.victims > 10:
        parser.error(f"--victims must be between 1 and 10, got {args.victims}")

    set_victim_config(victim_count=args.victims, seed=args.seed)

    sdf, metadata = world()

    with open(OUTPUT_SDF, "w") as f:
        f.write(sdf)

    with open(OUTPUT_META, "w") as f:
        json.dump(metadata, f, indent=4)

    print()
    print(f"Generated ({args.victims} victims, seed={args.seed})")
    print(OUTPUT_SDF)
    print(OUTPUT_META)

if __name__ == "__main__":
    main()