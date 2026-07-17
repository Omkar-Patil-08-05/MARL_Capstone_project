import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

RESULTS_DIR = os.path.expanduser("~/capstone_project/results")

# -------------------------------
# AUTO LOAD LATEST FILES
# -------------------------------
files = os.listdir(RESULTS_DIR)

cov_file = sorted([f for f in files if f.startswith("coverage_") and f.endswith(".csv")])[-1]
pos_file = sorted([f for f in files if f.startswith("positions_") and f.endswith(".csv")])[-1]

cov_path = os.path.join(RESULTS_DIR, cov_file)
pos_path = os.path.join(RESULTS_DIR, pos_file)

print("Using:")
print(cov_path)
print(pos_path)

# -------------------------------
# LOAD DATA
# -------------------------------
cov = pd.read_csv(cov_path) 
pos = pd.read_csv(pos_path)

# -------------------------------
# 1. COVERAGE GRAPH
# -------------------------------
plt.figure()

# smoothing
cov["smooth"] = cov["coverage"].rolling(window=5, min_periods=1).mean()

plt.plot(cov["time_step"], cov["coverage"], alpha=0.3, label="Raw")
plt.plot(cov["time_step"], cov["smooth"], linewidth=2, label="Smoothed")

plt.axhline(y=1.0, linestyle='--', label="Max Coverage")

plt.xlabel("Time Step")
plt.ylabel("Coverage")
plt.title("Coverage vs Time")
plt.ylim(0, 1.05)
plt.grid(True)
plt.legend()

plt.savefig(os.path.join(RESULTS_DIR, "coverage_plot.png"))
print("Saved coverage_plot.png")

# -------------------------------
# 2. HEATMAP 
# -------------------------------
grid_size = 25
heatmap = np.zeros((grid_size, grid_size))

for _, row in pos.iterrows():
    x = int(row["x"])
    y = int(row["y"])
    heatmap[y][x] += 1

plt.figure()
plt.imshow(heatmap, origin="lower")
plt.colorbar(label="Visit Frequency")

plt.title("Coverage Heatmap")
plt.xlabel("X")
plt.ylabel("Y")

plt.xticks(range(0, 25, 5))
plt.yticks(range(0, 25, 5))
plt.grid(True)

plt.savefig(os.path.join(RESULTS_DIR, "heatmap.png"))
print("Saved heatmap.png")

# -------------------------------
# 3. TRAJECTORIES
# -------------------------------
plt.figure()

for drone in pos["drone"].unique():
    d = pos[pos["drone"] == drone]

    plt.plot(d["x"], d["y"], linewidth=2, label=drone)

    # start point
    plt.scatter(d["x"].iloc[0], d["y"].iloc[0])

    # end point
    plt.scatter(d["x"].iloc[-1], d["y"].iloc[-1])

plt.legend()
plt.title("Drone Trajectories")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)

plt.savefig(os.path.join(RESULTS_DIR, "trajectories.png"))
print("Saved trajectories.png")

# -------------------------------
# 4. PER-DRONE COVERAGE
# -------------------------------
drone_coverage = {}

for drone in pos["drone"].unique():
    d = pos[pos["drone"] == drone]
    visited = set(zip(d["x"], d["y"]))
    drone_coverage[drone] = len(visited)

plt.figure()
plt.bar(drone_coverage.keys(), drone_coverage.values())

plt.title("Per-Drone Coverage (Unique Cells)")
plt.xlabel("Drone")
plt.ylabel("Cells Covered")

plt.savefig(os.path.join(RESULTS_DIR, "per_drone_coverage.png"))
print("Saved per_drone_coverage.png")

# -------------------------------
# 5. COVERAGE SPEED
# -------------------------------
plt.figure()

cov["speed"] = cov["coverage"].diff().fillna(0)

plt.plot(cov["time_step"], cov["speed"], linewidth=2)

plt.xlabel("Time Step")
plt.ylabel("Coverage Speed")
plt.title("Coverage Speed (dCoverage/dt)")
plt.grid(True)

plt.savefig(os.path.join(RESULTS_DIR, "coverage_speed.png"))
print("Saved coverage_speed.png")

# -------------------------------
# 6. HEATMAP WITH OBSTACLES 
# -------------------------------
plt.figure()

plt.imshow(heatmap, origin="lower")
plt.colorbar(label="Visit Frequency")

#  OBSTACLES (MATCH CONTROLLER)
obstacles = [
    (4, 4),
    (8, 6),
    (12, 10),
    (16, 14),
    (20, 18),
    (6, 18),
    (18, 6),
    (10, 20),
    (14, 8)
]

ox = [o[0] for o in obstacles]
oy = [o[1] for o in obstacles]

plt.scatter(ox, oy, marker="X", s=100)

plt.title("Coverage Heatmap with Obstacles")
plt.xlabel("X")
plt.ylabel("Y")

plt.grid(True)

plt.savefig(os.path.join(RESULTS_DIR, "heatmap_with_obstacles.png"))
print("Saved heatmap_with_obstacles.png")

# -------------------------------
# SHOW ALL
# -------------------------------
plt.show()