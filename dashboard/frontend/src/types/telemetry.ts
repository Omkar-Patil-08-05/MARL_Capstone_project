export interface DroneTelemetry {
    id: string;
    state: string;
    x: number;
    y: number;
    z: number;
    grid_x: number;
    grid_y: number;
    action: string;
    safety_override: boolean;
}

export interface MissionTelemetry {
    status: string;
    decision_count: number;
    max_decisions: number;
    global_step_state?: string;
    coverage: number;
    victims_detected: number;
    total_victims: number;
    explored_count: number;
    valid_count: number;
    safety_overrides: number;
}

export interface VictimState {
    id: string;
    x: number;
    y: number;
    world_x?: number;
    world_y?: number;
    detected: boolean;
    state?: string;
    source?: string;
    confidence?: number;
    detected_by?: string;
    last_seen_sec_ago?: number;
    gt_error?: number;
    observations?: number;
}

export interface ExploredCell {
    x: number;
    y: number;
}

export interface TelemetryPayload {
    type: string;
    timestamp: number;
    mission: MissionTelemetry;
    drones: DroneTelemetry[];
    explored_cells: ExploredCell[];
    victims: VictimState[];
    tracked_victims?: VictimState[];
    active_map_id?: string;
    backend_status?: string;
}

export interface WorldObstacle {
    id: string;
    type: string;
    aabb: {
        min_x: number;
        max_x: number;
        min_y: number;
        max_y: number;
    };
}

export interface WorldVictim {
    id: string;
    grid: {
        x: number;
        y: number;
    };
}

export interface WorldData {
    grid: {
        width: number;
        height: number;
        meters_per_cell: number;
    };
    world: {
        width_m: number;
        height_m: number;
        origin_x: number;
        origin_y: number;
    };
    obstacles: WorldObstacle[];
    victims: WorldVictim[];
    drone_spawns: { id: string; x: number; y: number }[];
}

export interface AlertEvent {
    id: string;
    timestamp: number;
    message: string;
    type: 'info' | 'warning' | 'success';
}

export interface MapRegistryEntry {
    id: string;
    name: string;
    world_file: string;
    metadata_file: string;
    grid_width: number;
    grid_height: number;
    meters_per_cell: number;
    victim_count: number;
    policy_compatible: boolean;
}

export type MapRegistry = Record<string, MapRegistryEntry>;

export interface BackendMissionStatus {
    state: 'IDLE' | 'STARTING' | 'SIMULATOR_READY' | 'QMIX_STARTING' | 'RUNNING' | 'COMPLETE' | 'STOPPING' | 'ERROR';
    active_map_id: string | null;
    error: string | null;
}
