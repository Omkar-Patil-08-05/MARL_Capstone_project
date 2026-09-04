#!/usr/bin/env python3
import os
import sqlite3
import csv
import json
import uuid
import time

import sys
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "dashboard", "backend"))
from database import DB_PATH  # type: ignore

def init_db():
    # Schema is initialized when database module is imported (db_writer instantiated)
    conn = sqlite3.connect(DB_PATH)
    return conn

def migrate_qmix_batch(conn):
    csv_path = os.path.join(os.path.dirname(__file__), "..", "gazebo_evaluation", "summary.csv")
    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        return

    experiment_id = str(uuid.uuid4())
    metadata = {
        "notes": "Historical QMIX batch. Reported total 20 episodes. Raw CSV available: 10 episodes (episodes 11-20). Discrepancy documented in final data audit.",
        "drone_count": 6,
        "qmix_checkpoint": "qmix_sar_v4_align_best.pth",
        "victim_count": 5
    }

    cursor = conn.cursor()
    cursor.execute(
        "INSERT INTO experiments (id, type, name, timestamp, environment, qmix_checkpoint, config_metadata) VALUES (?, ?, ?, ?, ?, ?, ?)",
        (experiment_id, "EVALUATION", "Valid QMIX Evaluation Batch (Partial CSV)", time.time(), "realistic_sar", "qmix_sar_v4_align_best.pth", json.dumps(metadata))
    )

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            mission_id = str(uuid.uuid4())
            episode_id = str(uuid.uuid4())
            
            # Extract fields
            episode_num = int(row.get("episode_id", 0))
            seed = int(row.get("seed", 0))
            duration = float(row.get("duration", 0))
            coverage = float(row.get("coverage", 0))
            victims_found = int(row.get("victims_found", 0))
            hover_actions = int(row.get("hover_actions", 0))
            
            start_time = time.time() - duration
            end_time = time.time()
            
            cursor.execute(
                "INSERT INTO missions (id, experiment_id, map_id, drone_count, victim_count, random_seed, start_time, end_time, status, final_coverage, safety_overrides) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (mission_id, experiment_id, "realistic_sar", 6, 5, seed, start_time, end_time, "COMPLETE", coverage, hover_actions)
            )

            cursor.execute(
                "INSERT INTO episodes (id, mission_id, episode_number, start_time, end_time, duration, coverage, victims_found, hover_actions, status) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (episode_id, mission_id, episode_num, start_time, end_time, duration, coverage, victims_found, hover_actions, "COMPLETE")
            )

    conn.commit()
    print(f"Migrated valid QMIX evaluation batch (Episodes 11-20) into experiment {experiment_id}")

def migrate_historical_benchmarks(conn):
    # Historical monocular / RGB-D benchmark (stage1 vs stage4)
    experiment_id = str(uuid.uuid4())
    metadata = {
        "notes": "Historical perception experiments. Distinct from QMIX flight control evaluation.",
        "stage1_mean_reward": 13.044143491452445,
        "stage1_max_reward": 19.349609375,
        "stage4_mean_reward": 14.154130094760173,
        "stage4_max_reward": 19.82421875
    }
    
    cursor = conn.cursor()
    cursor.execute(
        "INSERT INTO experiments (id, type, name, timestamp, environment, config_metadata) VALUES (?, ?, ?, ?, ?, ?)",
        (experiment_id, "HISTORICAL", "Historical Perception Benchmarks (Stage 1 vs 4)", time.time(), "various", json.dumps(metadata))
    )
    conn.commit()
    print(f"Migrated historical perception benchmarks into experiment {experiment_id}")

def main():
    print("Starting historical data migration...")
    conn = init_db()
    
    cursor = conn.cursor()
    cursor.execute("SELECT count(*) FROM experiments WHERE name = 'Valid QMIX Evaluation Batch (Partial CSV)'")
    if cursor.fetchone()[0] > 0:
        print("Migration already run. Skipping.")
        conn.close()
        return

    migrate_qmix_batch(conn)
    migrate_historical_benchmarks(conn)
    
    conn.close()
    print("Migration complete.")

if __name__ == "__main__":
    main()
