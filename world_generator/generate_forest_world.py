import argparse
import os
from forest_config import set_forest_seed, DEFAULT_TREE_COUNT, DEFAULT_CAST_SHADOWS, VICTIM_GROUND_OFFSET
from forest_generator import generate_forest_world

DEFAULT_OUTPUT_DIR = os.path.expanduser("~/capstone_project_antigravity/worlds")

def main():
    parser = argparse.ArgumentParser(description="Generate MARL Forest SAR world.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for generation.")
    parser.add_argument("--victims", type=int, default=5, help="Number of victims to spawn.")
    parser.add_argument("--trees", type=int, default=DEFAULT_TREE_COUNT,
                        help=f"Number of trees to generate (default: {DEFAULT_TREE_COUNT} for Stage 1 baseline).")
    parser.add_argument("--shadows", action="store_true", default=DEFAULT_CAST_SHADOWS,
                        help="Enable directional sun shadows (default: False to protect GPU).")
    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR,
                        help=f"Output directory for generated files (default: {DEFAULT_OUTPUT_DIR}).")
    parser.add_argument("--output-name", type=str, default="forest_sar",
                        help="Base filename for generated outputs (default: forest_sar).")
    parser.add_argument("--victim-offset", type=float, default=VICTIM_GROUND_OFFSET,
                        help=f"Vertical offset in meters for victim ground contact (default: {VICTIM_GROUND_OFFSET}m).")
    args = parser.parse_args()
    
    out_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    os.makedirs(out_dir, exist_ok=True)
    
    set_forest_seed(args.seed)
    
    # Protected baseline safeguard
    protected_baselines = [
        "forest_stage1_baseline.sdf",
        "forest_stage1_baseline_metadata.json",
        "forest_rescue_world.sdf",
        "forest_rescue_metadata.json",
        "earthquake_world.sdf"
    ]
    if f"{args.output_name}.sdf" in protected_baselines and "generated_worlds" in out_dir:
        raise RuntimeError(f"Refusing to overwrite protected baseline world: {args.output_name} in {out_dir}")
    
    out_sdf = os.path.join(out_dir, f"{args.output_name}.sdf")
    out_map = os.path.join(out_dir, f"{args.output_name}_map.png")
    out_heightmap = os.path.join(out_dir, f"{args.output_name}_heightmap.png")
    out_meta = os.path.join(out_dir, f"{args.output_name}_meta.json")
    
    metadata = generate_forest_world(
        seed=args.seed,
        victims_count=args.victims,
        out_sdf=out_sdf,
        out_map=out_map,
        out_heightmap=out_heightmap,
        out_meta=out_meta,
        tree_count=args.trees,
        cast_shadows=args.shadows,
        victim_ground_offset=args.victim_offset
    )
    
    print(f"\nSuccessfully generated forest world '{args.output_name}' with seed {args.seed}!")
    print(f"SDF File: {out_sdf}")
    print(f"Map Image: {out_map}")
    print(f"Heightmap: {out_heightmap}")
    print(f"Metadata: {out_meta}")

if __name__ == "__main__":
    main()
