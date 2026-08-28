import { useState, useEffect, useRef } from 'react';
import type { TelemetryPayload, WorldData, AlertEvent, BackendMissionStatus, MapRegistry } from '../types/telemetry';

const WS_URL = 'ws://localhost:8000/ws/telemetry';
const API_URL = 'http://localhost:8000/api';

export function useTelemetry() {
    const [telemetry, setTelemetry] = useState<TelemetryPayload | null>(null);
    const [worldData, setWorldData] = useState<WorldData | null>(null);
    const [mapRegistry, setMapRegistry] = useState<MapRegistry | null>(null);
    const [backendStatus, setBackendStatus] = useState<BackendMissionStatus>({ state: 'IDLE', active_map_id: null, error: null });
    const [isConnected, setIsConnected] = useState(false);
    const [alerts, setAlerts] = useState<AlertEvent[]>([]);
    
    // We keep track of history for the map
    const [droneHistory, setDroneHistory] = useState<Record<string, {x: number, y: number}[]>>({});
    
    const wsRef = useRef<WebSocket | null>(null);
    const reconnectTimeout = useRef<number | null>(null);
    const prevTelemetry = useRef<TelemetryPayload | null>(null);

    // Fetch static data and mission status
    useEffect(() => {
        fetch(`${API_URL}/maps`)
            .then(res => res.json())
            .then(data => setMapRegistry(data))
            .catch(err => console.error("Failed to load map registry", err));
            
        const pollStatus = () => {
            fetch(`${API_URL}/mission/status`)
                .then(res => res.json())
                .then(data => setBackendStatus(data))
                .catch(err => console.error("Failed to fetch mission status", err));
        };
        
        pollStatus();
        const interval = setInterval(pollStatus, 1000);
        return () => clearInterval(interval);
    }, []);
    
    // Auto-fetch world data when an active map ID is set
    useEffect(() => {
        if (backendStatus.active_map_id) {
            fetch(`${API_URL}/maps/${backendStatus.active_map_id}`)
                .then(res => res.json())
                .then(data => {
                    if (!data.error) setWorldData(data);
                    else console.error("World API Error:", data.error);
                })
                .catch(err => console.error("Failed to load world data", err));
        } else {
            setWorldData(null);
            setTelemetry(null);
            setDroneHistory({});
            setAlerts([]);
        }
    }, [backendStatus.active_map_id]);

    // WebSocket logic
    useEffect(() => {
        connect();
        return () => {
            if (wsRef.current) {
                wsRef.current.onclose = null; // Prevent reconnect loop
                wsRef.current.close();
            }
            if (reconnectTimeout.current) {
                clearTimeout(reconnectTimeout.current);
            }
        };
    }, []);

    const connect = () => {
        try {
            const ws = new WebSocket(WS_URL);
            
            ws.onopen = () => {
                setIsConnected(true);
            };

            ws.onmessage = (event) => {
                try {
                    const data: TelemetryPayload = JSON.parse(event.data);
                    if (data.type === 'telemetry') {
                        processTelemetryUpdate(data);
                        setTelemetry(data);
                    }
                } catch (e) {
                    console.error("Failed to parse telemetry", e);
                }
            };

            ws.onclose = () => {
                setIsConnected(false);
                // Attempt to reconnect
                reconnectTimeout.current = window.setTimeout(connect, 3000);
            };

            wsRef.current = ws;
        } catch (e) {
            console.error("WebSocket setup error", e);
        }
    };

    const processTelemetryUpdate = (current: TelemetryPayload) => {
        const prev = prevTelemetry.current;
        
        // Apply EMA Smoothing to coordinates (alpha = 0.3)
        if (prev) {
            const alpha = 0.3;
            current.drones.forEach(d => {
                const pd = prev.drones.find(x => x.id === d.id);
                if (pd) {
                    d.x = alpha * d.x + (1 - alpha) * pd.x;
                    d.y = alpha * d.y + (1 - alpha) * pd.y;
                    d.z = alpha * d.z + (1 - alpha) * pd.z;
                }
            });
        }
        
        // 1. Update Trajectory History
        setDroneHistory(oldHistory => {
            const newHistory = { ...oldHistory };
            current.drones.forEach(d => {
                if (!newHistory[d.id]) newHistory[d.id] = [];
                // Only append if it moved significantly to save memory, or just append everything and slice
                const last = newHistory[d.id][newHistory[d.id].length - 1];
                if (!last || last.x !== d.x || last.y !== d.y) {
                    newHistory[d.id] = [...newHistory[d.id], {x: d.x, y: d.y}].slice(-200); // keep last 200 points
                }
            });
            return newHistory;
        });

        // 2. Generate Events
        if (prev) {
            const newAlerts: AlertEvent[] = [];
            
            // Mission status change
            if (prev.mission.status !== current.mission.status) {
                newAlerts.push({
                    id: Math.random().toString(),
                    timestamp: current.timestamp,
                    message: `Mission entered ${current.mission.status}`,
                    type: current.mission.status === 'ERROR' ? 'warning' : 'info'
                });
            }

            // Victim detection — identify WHICH victim was detected
            if (current.victims && prev.victims) {
                current.victims.forEach(v => {
                    const pv = prev.victims.find(x => x.id === v.id);
                    if (v.detected && pv && !pv.detected) {
                        // Find the closest drone at this moment
                        let closestDrone = 'SWARM';
                        let minDist = Infinity;
                        current.drones.forEach(d => {
                            const dx = d.grid_x - v.x;
                            const dy = d.grid_y - v.y;
                            const dist = Math.sqrt(dx * dx + dy * dy);
                            if (dist < minDist) {
                                minDist = dist;
                                closestDrone = d.id.replace('drone_', 'DRONE ');
                            }
                        });
                        newAlerts.push({
                            id: Math.random().toString(),
                            timestamp: current.timestamp,
                            message: `🚁 ${closestDrone} DETECTED VICTIM ${v.id} at (${v.x}, ${v.y})`,
                            type: 'success'
                        });
                    }
                });
            }

            // Coverage milestones (every 10%)
            const prevCov = Math.floor(prev.mission.coverage / 10);
            const currCov = Math.floor(current.mission.coverage / 10);
            if (currCov > prevCov && current.mission.coverage > 0) {
                newAlerts.push({
                    id: Math.random().toString(),
                    timestamp: current.timestamp,
                    message: `📊 SEARCH COVERAGE ${Math.floor(current.mission.coverage)}%`,
                    type: 'info'
                });
            }

            // Drone states & overrides
            current.drones.forEach(d => {
                const pd = prev.drones.find(x => x.id === d.id);
                if (pd) {
                    if (pd.state !== d.state) {
                        newAlerts.push({
                            id: Math.random().toString(),
                            timestamp: current.timestamp,
                            message: `${d.id.replace('_', ' ').toUpperCase()} → ${d.state}`,
                            type: 'info'
                        });
                    }
                    if (!pd.safety_override && d.safety_override) {
                        newAlerts.push({
                            id: Math.random().toString(),
                            timestamp: current.timestamp,
                            message: `🚨 ${d.id.replace('_', ' ').toUpperCase()} SAFETY OVERRIDE: ${d.action}`,
                            type: 'warning'
                        });
                    }
                }
            });

            if (newAlerts.length > 0) {
                setAlerts(old => [...newAlerts, ...old].slice(0, 30)); // keep last 30
            }
        }

        prevTelemetry.current = current;
    };
    
    const startMission = async (mapId: string, droneCount: number = 2) => {
        try {
            await fetch(`${API_URL}/mission/start`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ map_id: mapId, drone_count: droneCount })
            });
        } catch (e) {
            console.error(e);
        }
    };
    
    const stopMission = async () => {
        try {
            await fetch(`${API_URL}/mission/stop`, { method: 'POST' });
        } catch (e) {
            console.error(e);
        }
    };
    
    const resetMission = async () => {
        try {
            await fetch(`${API_URL}/mission/reset`, { method: 'POST' });
        } catch (e) {
            console.error(e);
        }
    };

    return {
        telemetry,
        worldData,
        mapRegistry,
        backendStatus,
        isConnected,
        alerts,
        droneHistory,
        startMission,
        stopMission,
        resetMission
    };
}
