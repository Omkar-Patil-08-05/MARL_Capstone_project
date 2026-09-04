import sqlite3
import os
import json
import threading
import queue
import time
from enum import Enum

DB_PATH = os.path.join(os.path.dirname(__file__), "results", "antigravity.db")

class EventType(Enum):
    EXPERIMENT = "EXPERIMENT"
    MISSION_START = "MISSION_START"
    MISSION_END = "MISSION_END"
    EPISODE = "EPISODE"
    DRONE = "DRONE"
    VICTIM = "VICTIM"
    DETECTION = "DETECTION"
    TELEMETRY = "TELEMETRY"
    PERFORMANCE = "PERFORMANCE"
    SAFETY = "SAFETY"

class DatabaseWriter(threading.Thread):
    def __init__(self, db_path=DB_PATH, max_qsize=50000, batch_size=1000):
        super().__init__(daemon=True, name="DatabaseWriterThread")
        self.db_path = db_path
        self.queue = queue.Queue(maxsize=max_qsize)
        self.batch_size = batch_size
        self.stop_event = threading.Event()
        self.conn = None
        self._init_db()
        
    def _init_db(self):
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.conn.execute("PRAGMA journal_mode=WAL;")
        self.conn.execute("PRAGMA synchronous=NORMAL;")
        self.conn.execute("PRAGMA foreign_keys=ON;")
        self.conn.execute("PRAGMA busy_timeout=5000;")
        
        cursor = self.conn.cursor()
        
        # EXPERIMENTS
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS experiments (
            id TEXT PRIMARY KEY,
            type TEXT,
            name TEXT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            environment TEXT,
            qmix_checkpoint TEXT,
            config_metadata TEXT
        )
        """)
        
        # MISSIONS
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS missions (
            id TEXT PRIMARY KEY,
            experiment_id TEXT REFERENCES experiments(id),
            map_id TEXT,
            drone_count INTEGER,
            victim_count INTEGER,
            perception_mode TEXT,
            detection_radius REAL,
            random_seed INTEGER,
            start_time REAL,
            end_time REAL,
            status TEXT,
            final_coverage REAL,
            safety_overrides INTEGER
        )
        """)
        
        # EPISODES
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS episodes (
            id TEXT PRIMARY KEY,
            mission_id TEXT REFERENCES missions(id) ON DELETE CASCADE,
            episode_number INTEGER,
            start_time REAL,
            end_time REAL,
            duration REAL,
            coverage REAL,
            victims_found INTEGER,
            total_distance REAL,
            hover_actions INTEGER,
            collision_count INTEGER,
            timeout_count INTEGER,
            invalid_flag BOOLEAN,
            status TEXT
        )
        """)
        
        # DRONES
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS drones (
            id TEXT PRIMARY KEY,
            episode_id TEXT REFERENCES episodes(id) ON DELETE CASCADE,
            drone_index INTEGER,
            model_config TEXT,
            final_state TEXT,
            final_x REAL,
            final_y REAL,
            final_z REAL,
            final_vx REAL,
            final_vy REAL,
            final_vz REAL
        )
        """)
        
        # VICTIMS
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS victims (
            id TEXT PRIMARY KEY,
            mission_id TEXT REFERENCES missions(id) ON DELETE CASCADE,
            world_x REAL,
            world_y REAL,
            grid_x INTEGER,
            grid_y INTEGER,
            detection_status TEXT
        )
        """)
        
        # DETECTION EVENTS
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS detection_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            mission_id TEXT REFERENCES missions(id) ON DELETE CASCADE,
            episode_id TEXT REFERENCES episodes(id) ON DELETE CASCADE,
            victim_id TEXT,
            drone_id TEXT,
            timestamp REAL,
            detection_source TEXT,
            detection_world_x REAL,
            detection_world_y REAL,
            detection_world_z REAL,
            euclidean_distance REAL,
            confidence REAL,
            confirmation_count INTEGER,
            UNIQUE(mission_id, victim_id, detection_source)
        )
        """)
        
        # TELEMETRY
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            mission_id TEXT REFERENCES missions(id) ON DELETE CASCADE,
            episode_id TEXT REFERENCES episodes(id) ON DELETE CASCADE,
            drone_id TEXT,
            x REAL, y REAL, z REAL,
            vx REAL, vy REAL, vz REAL,
            state TEXT,
            action TEXT
        )
        """)
        
        # PERFORMANCE
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS performance_samples (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            mission_id TEXT REFERENCES missions(id) ON DELETE CASCADE,
            episode_id TEXT REFERENCES episodes(id) ON DELETE CASCADE,
            rtf REAL,
            cpu_utilization REAL,
            memory_utilization REAL,
            gpu_utilization REAL,
            gpu_memory REAL
        )
        """)
        
        # SAFETY EVENTS
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS safety_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            mission_id TEXT REFERENCES missions(id) ON DELETE CASCADE,
            episode_id TEXT REFERENCES episodes(id) ON DELETE CASCADE,
            drone_ids TEXT,
            conflict_type TEXT,
            action_taken TEXT,
            metadata TEXT
        )
        """)
        
        # Indexes for fast querying
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_mission ON telemetry(mission_id)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_drone ON telemetry(drone_id)")
        
        self.conn.commit()
        
    def enqueue(self, event_type: EventType, data: dict):
        try:
            if event_type in [EventType.TELEMETRY, EventType.PERFORMANCE]:
                # Drop if queue is full to prioritize control/lifecycle
                self.queue.put_nowait((event_type, data))
            else:
                # Block for critical events
                self.queue.put((event_type, data), timeout=2.0)
        except queue.Full:
            print(f"[DB_WRITER] WARNING: Queue full. Dropped {event_type.name} event.")

    def run(self):
        print(f"[DB_WRITER] Started. Writing to {self.db_path}")
        batch = []
        last_commit_time = time.time()
        
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                # Use a small timeout so we can check stop_event periodically
                item = self.queue.get(timeout=1.0)
                batch.append(item)
            except queue.Empty:
                pass
            
            # Commit if batch is large enough or time has elapsed
            if len(batch) >= self.batch_size or (len(batch) > 0 and time.time() - last_commit_time > 1.0):
                self._flush_batch(batch)
                batch.clear()
                last_commit_time = time.time()
                
        # Final flush on shutdown
        if batch:
            self._flush_batch(batch)
            
        if self.conn:
            self.conn.close()
            print("[DB_WRITER] Stopped cleanly.")
            
    def _flush_batch(self, batch):
        try:
            cursor = self.conn.cursor()
            
            telemetry_args = []
            perf_args = []
            
            for event_type, data in batch:
                if event_type == EventType.EXPERIMENT:
                    cursor.execute(
                        "INSERT OR IGNORE INTO experiments (id, type, name, timestamp, environment, qmix_checkpoint, config_metadata) VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (data['id'], data.get('type'), data.get('name'), data.get('timestamp'), data.get('environment'), data.get('qmix_checkpoint'), data.get('config_metadata'))
                    )
                elif event_type == EventType.MISSION_START:
                    cursor.execute(
                        "INSERT OR IGNORE INTO missions (id, experiment_id, map_id, drone_count, victim_count, perception_mode, detection_radius, random_seed, start_time, status) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                        (data['id'], data.get('experiment_id'), data.get('map_id'), data.get('drone_count'), data.get('victim_count'), data.get('perception_mode'), data.get('detection_radius'), data.get('random_seed'), data.get('start_time'), data.get('status'))
                    )
                elif event_type == EventType.MISSION_END:
                    cursor.execute(
                        "UPDATE missions SET end_time = ?, status = ?, final_coverage = ?, safety_overrides = ? WHERE id = ?",
                        (data.get('end_time'), data.get('status'), data.get('final_coverage'), data.get('safety_overrides'), data['id'])
                    )
                elif event_type == EventType.EPISODE:
                    cursor.execute(
                        "INSERT OR IGNORE INTO episodes (id, mission_id, episode_number, start_time, end_time, duration, coverage, victims_found, total_distance, hover_actions, collision_count, timeout_count, invalid_flag, status) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                        (data['id'], data['mission_id'], data.get('episode_number'), data.get('start_time'), data.get('end_time'), data.get('duration'), data.get('coverage'), data.get('victims_found'), data.get('total_distance'), data.get('hover_actions'), data.get('collision_count'), data.get('timeout_count'), data.get('invalid_flag', False), data.get('status'))
                    )
                elif event_type == EventType.DRONE:
                    cursor.execute(
                        "INSERT OR IGNORE INTO drones (id, episode_id, drone_index, model_config, final_state, final_x, final_y, final_z, final_vx, final_vy, final_vz) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                        (data['id'], data.get('episode_id'), data.get('drone_index'), data.get('model_config'), data.get('final_state'), data.get('final_x'), data.get('final_y'), data.get('final_z'), data.get('final_vx'), data.get('final_vy'), data.get('final_vz'))
                    )
                elif event_type == EventType.VICTIM:
                    cursor.execute(
                        "INSERT OR IGNORE INTO victims (id, mission_id, world_x, world_y, grid_x, grid_y, detection_status) VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (data['id'], data['mission_id'], data.get('world_x'), data.get('world_y'), data.get('grid_x'), data.get('grid_y'), data.get('detection_status'))
                    )
                elif event_type == EventType.DETECTION:
                    cursor.execute(
                        "INSERT OR IGNORE INTO detection_events (mission_id, episode_id, victim_id, drone_id, timestamp, detection_source, detection_world_x, detection_world_y, detection_world_z, euclidean_distance, confidence, confirmation_count) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                        (data['mission_id'], data.get('episode_id'), data['victim_id'], data.get('drone_id'), data.get('timestamp'), data.get('detection_source'), data.get('detection_world_x'), data.get('detection_world_y'), data.get('detection_world_z'), data.get('euclidean_distance'), data.get('confidence'), data.get('confirmation_count'))
                    )
                elif event_type == EventType.SAFETY:
                    cursor.execute(
                        "INSERT INTO safety_events (timestamp, mission_id, episode_id, drone_ids, conflict_type, action_taken, metadata) VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (data.get('timestamp'), data.get('mission_id'), data.get('episode_id'), data.get('drone_ids'), data.get('conflict_type'), data.get('action_taken'), data.get('metadata'))
                    )
                elif event_type == EventType.TELEMETRY:
                    telemetry_args.append((
                        data.get('timestamp'), data.get('mission_id'), data.get('episode_id'), data.get('drone_id'),
                        data.get('x'), data.get('y'), data.get('z'), data.get('vx'), data.get('vy'), data.get('vz'),
                        data.get('state'), data.get('action')
                    ))
                elif event_type == EventType.PERFORMANCE:
                    perf_args.append((
                        data.get('timestamp'), data.get('mission_id'), data.get('episode_id'),
                        data.get('rtf'), data.get('cpu_utilization'), data.get('memory_utilization'),
                        data.get('gpu_utilization'), data.get('gpu_memory')
                    ))
            
            # Batch inserts for high volume data
            if telemetry_args:
                cursor.executemany(
                    "INSERT INTO telemetry (timestamp, mission_id, episode_id, drone_id, x, y, z, vx, vy, vz, state, action) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    telemetry_args
                )
                
            if perf_args:
                cursor.executemany(
                    "INSERT INTO performance_samples (timestamp, mission_id, episode_id, rtf, cpu_utilization, memory_utilization, gpu_utilization, gpu_memory) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                    perf_args
                )
                
            self.conn.commit()
            
        except Exception as e:
            print(f"[DB_WRITER] ERROR: Transaction failed: {e}")
            if self.conn:
                self.conn.rollback()

    def shutdown(self):
        self.stop_event.set()
        self.join()
        
db_writer = DatabaseWriter()
db_writer.start()
