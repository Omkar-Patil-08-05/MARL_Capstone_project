# Phase N3: Read-Only Architectural Inspection (earthquake_world.sdf)

This is a strictly read-only diagnostic report evaluating the architectural requirements to launch the unvalidated `earthquake_world.sdf` environment using the existing 2-drone QMIX policy.

---

## 1. Current Architecture Summary
The existing system (`qmix_sar_v4_align_best.pth`) is explicitly hardcoded and validated exclusively for the `realistic_sar` environment (25x25).
The physical architecture relies on a `generated_world_meta.json` file which provides ground-truth drone spawn coordinates, grid sizing, and semantic victim locations required by `qmix_drone_test.py` and `launch_two_drones.sh`.

---

## 2. earthquake_world.sdf Facts
A strict inspection reveals the following about the larger unvalidated environment:
- **Existence:** The file successfully exists at `worlds/earthquake_world.sdf`.
- **Size & Complexity:** It is massively larger than the validated world.
  - File Size: **372 KB** (vs 47 KB for `realistic_sar.sdf`)
  - Total Models: **570** (vs 87)
  - Visual Meshes: **981** (vs 87)
  - Collision Meshes: **1** (Aggregated heightmap/mesh collision)
- **Metadata Consistency:** **FAIL**. There is currently NO dedicated metadata JSON file (e.g., `earthquake_world_meta.json`) in the repository. The environment lacks explicitly defined semantic drone spawn coordinates, grid dimensions, and victim coordinates required by the Python and Bash controllers.

---

## 3. Current Map-Selection Architecture
- **Backend Registry (`dashboard/backend/world.py`):** Explicitly hardcoded to only expose `"realistic_sar"` in the `MAP_REGISTRY` dictionary.
- **Frontend UI (`dashboard/frontend/src/components/MapSelection.tsx`):** Dynamically populates from the backend registry. Because the backend omits it, `earthquake_world` cannot currently be clicked.
- **Launch Script (`scripts/launch_two_drones.sh`):** Contains an explicit `if [ "$MAP_ID" == "realistic_sar" ]; then... else exit 1`.
- **QMIX Entry (`qmix_drone_test.py`):** Contains an explicit `if map_id == "realistic_sar": ... else raise ValueError`.

---

## 4. Exact Compatibility Assessment
Can we validate `earthquake_world` with the existing 2-drone policy WITHOUT retraining?

1. **Architectural Compatibility (PyTorch): YES.**
   The QMIX observation vector (29-D) is constructed using normalized coordinates (0.0 to 1.0) and fixed-size local 3x3 FOV grids. Therefore, a larger grid size will not change the 29-D tensor shape, meaning the PyTorch forward pass will **not** crash.
2. **Policy Compatibility (RL Behavior): UNKNOWN / THEORETICAL.**
   Because global coordinates are normalized `x / x_size`, a distance of `0.1` means 2.5 cells in a 25x25 grid but 5.0 cells in a 50x50 grid. The QMIX agent may exhibit degraded spatial generalization, but it *can* mathematically execute without retraining.
3. **Physical Simulation Compatibility (Gazebo): YES.**
   Headless Gazebo will effortlessly load the 570 models.
4. **Quantitative Validation: NONE.**
   The environment is strictly unvalidated.

---

## 5. Exact Files Requiring Modification
To make `earthquake_world` a natively selectable dashboard map, the following **exact** modifications are required:

1. **`worlds/earthquake_world_meta.json`**: Must be generated/created to provide `"grid_width"`, `"grid_height"`, and `"drone_base": {"spawns": [...]}`.
2. **`dashboard/backend/world.py`**: Add `"earthquake_world"` to `MAP_REGISTRY`.
3. **`scripts/launch_two_drones.sh`**: Add an `elif [ "$MAP_ID" == "earthquake_world" ];` block to parse its respective metadata file for spawn points.
4. **`drone_ws/src/swarm_controller/swarm_controller/qmix_drone_test.py`**: Add an `elif map_id == "earthquake_world":` block to load its metadata file.

---

## 6. Proposed Safest Implementation Order
1. **Generate Metadata:** Run the world generator (or manually craft the JSON) to produce `earthquake_world_meta.json` with valid spawn points that do not clip into the 570 models.
2. **Modify Python Controllers:** Update `qmix_drone_test.py` to accept the ID.
3. **Modify Bash Controller:** Update `launch_two_drones.sh` to accept the ID.
4. **Modify Dashboard Registry:** Add the map to `world.py` so the UI exposes it.

---

## 7. Performance & Risks Analysis
**Real-Time Factor (RTF) Impact:**
The leap from 87 models to 570 models is substantial. However, running in **Headless Gazebo** mitigates the rendering pipeline cost entirely (saving immense GPU overhead). The physics engine will still need to compute collisions for the larger terrain mesh. We anticipate a minor RTF drop (e.g., from 1.0 to ~0.8), but it should easily remain stable enough for the 20Hz PX4 flight controllers.

**Collision Risk:**
If the generated spawn points in the new metadata file overlap with any of the 570 models, the drones will violently explode/crash upon Gazebo initialization before QMIX even takes control. Spawn point validation is critical.
