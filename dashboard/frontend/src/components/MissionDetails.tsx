import React, { useEffect, useState } from 'react';
import { TelemetryTable } from './TelemetryTable';
import type { Mission } from './ExperimentList';

interface Episode {
    id: string;
    episode_number: number;
    duration: number;
    coverage: number;
    victims_found: number;
    timeout_count: number;
    invalid_flag: boolean;
}

interface Victim {
    id: string;
    grid_x: number;
    grid_y: number;
    world_x: number;
    world_y: number;
    detection_status: string;
}

interface Detection {
    id: number;
    victim_id: string;
    drone_id: string;
    detection_source: string;
    timestamp: number;
    euclidean_distance: number;
    detection_world_x: number;
    detection_world_y: number;
}

interface SafetyEvent {
    id: number;
    timestamp: number;
    drone_ids: string;
    conflict_type: string;
    action_taken: string;
    metadata: string;
}

interface Props {
    missionId: string;
    onBack: () => void;
}

export function MissionDetails({ missionId, onBack }: Props) {
    const [mission, setMission] = useState<Mission | null>(null);
    const [episodes, setEpisodes] = useState<Episode[]>([]);
    const [victims, setVictims] = useState<Victim[]>([]);
    const [detections, setDetections] = useState<Detection[]>([]);
    const [safety, setSafety] = useState<SafetyEvent[]>([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);
    const [activeTab, setActiveTab] = useState<'EPISODES' | 'VICTIMS' | 'SAFETY' | 'TELEMETRY'>('EPISODES');

    useEffect(() => {
        setLoading(true);
        Promise.all([
            fetch(`http://localhost:8000/api/db/missions/${missionId}`).then(r => r.json()),
            fetch(`http://localhost:8000/api/db/episodes?mission_id=${missionId}`).then(r => r.json()),
            fetch(`http://localhost:8000/api/db/victims?mission_id=${missionId}`).then(r => r.json()),
            fetch(`http://localhost:8000/api/db/detections?mission_id=${missionId}`).then(r => r.json()),
            fetch(`http://localhost:8000/api/db/safety?mission_id=${missionId}`).then(r => r.json()),
        ]).then(([m, ep, v, d, s]) => {
            if (m.detail) throw new Error(m.detail);
            setMission(m);
            setEpisodes(ep);
            setVictims(v);
            setDetections(d);
            setSafety(s);
        }).catch(err => setError(err.message))
          .finally(() => setLoading(false));
    }, [missionId]);

    if (loading) return <div className="panel flex-col" style={{ flex: 1 }}>Loading mission details...</div>;
    if (error) return <div className="panel flex-col" style={{ flex: 1, color: 'var(--text-warning)' }}>Error: {error}</div>;
    if (!mission) return null;

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '16px', flex: 1, minHeight: 0 }}>
            {/* Header / Summary */}
            <div className="panel">
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                    <div className="panel-title">MISSION SUMMARY</div>
                    <button 
                        onClick={onBack}
                        style={{ background: 'transparent', border: '1px solid var(--text-muted)', color: 'white', padding: '4px 12px', borderRadius: '4px', cursor: 'pointer' }}
                    >
                        Back to List
                    </button>
                </div>
                
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '16px', marginTop: '16px' }}>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>ID</div>
                        <div style={{ fontSize: '0.9rem' }}>{mission.id.split('-')[0]}</div>
                    </div>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>Status</div>
                        <div style={{ color: mission.status === 'COMPLETE' ? 'var(--accent-green)' : 'var(--text-main)' }}>{mission.status}</div>
                    </div>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>Map</div>
                        <div>{mission.map_id}</div>
                    </div>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>Drones / Victims</div>
                        <div>{mission.drone_count} / {mission.victim_count}</div>
                    </div>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>Final Coverage</div>
                        <div style={{ color: 'var(--accent-cyan)' }}>{mission.final_coverage !== null ? `${mission.final_coverage}%` : 'N/A'}</div>
                    </div>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>Duration</div>
                        <div>{mission.end_time && mission.start_time ? `${(mission.end_time - mission.start_time).toFixed(1)}s` : 'N/A'}</div>
                    </div>
                    <div>
                        <div className="text-muted" style={{ fontSize: '0.8rem' }}>Safety Overrides</div>
                        <div style={{ color: mission.safety_overrides ? 'var(--text-warning)' : 'var(--text-main)' }}>{mission.safety_overrides ?? 0}</div>
                    </div>
                </div>
            </div>

            {/* Tabs */}
            <div style={{ display: 'flex', gap: '8px' }}>
                {['EPISODES', 'VICTIMS', 'SAFETY', 'TELEMETRY'].map(tab => (
                    <button
                        key={tab}
                        onClick={() => setActiveTab(tab as any)}
                        style={{
                            background: activeTab === tab ? 'var(--accent-cyan)' : 'var(--panel-bg)',
                            color: activeTab === tab ? 'black' : 'var(--text-muted)',
                            border: '1px solid ' + (activeTab === tab ? 'var(--accent-cyan)' : 'rgba(255,255,255,0.1)'),
                            padding: '6px 16px',
                            borderRadius: '4px',
                            cursor: 'pointer',
                            fontWeight: 'bold',
                            fontSize: '0.85rem'
                        }}
                    >
                        {tab}
                    </button>
                ))}
            </div>

            {/* Tab Content */}
            <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
                {activeTab === 'EPISODES' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">EPISODES</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem', marginTop: '12px' }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.2)', textAlign: 'left' }}>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Ep #</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Duration</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Coverage</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Victims</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Invalid</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Timeouts</th>
                                </tr>
                            </thead>
                            <tbody>
                                {episodes.map(e => (
                                    <tr key={e.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                                        <td style={{ padding: '8px 4px' }}>{e.episode_number ?? 'N/A'}</td>
                                        <td style={{ padding: '8px 4px' }}>{e.duration?.toFixed(1) ?? 'N/A'}s</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--accent-cyan)' }}>{e.coverage?.toFixed(2) ?? 'N/A'}%</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--accent-green)' }}>{e.victims_found ?? 0}</td>
                                        <td style={{ padding: '8px 4px' }}>{e.invalid_flag ? 'Yes' : 'No'}</td>
                                        <td style={{ padding: '8px 4px' }}>{e.timeout_count ?? 0}</td>
                                    </tr>
                                ))}
                                {episodes.length === 0 && <tr><td colSpan={6} style={{ padding: '16px', textAlign: 'center', color: 'var(--text-muted)' }}>No episodes recorded.</td></tr>}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'VICTIMS' && (
                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '16px', flex: 1, minHeight: 0 }}>
                        <div className="panel flex-col" style={{ overflowY: 'auto' }}>
                            <div className="panel-title">VICTIMS</div>
                            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem', marginTop: '12px' }}>
                                <thead>
                                    <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.2)', textAlign: 'left' }}>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>ID</th>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>World (X, Y)</th>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>State</th>
                                    </tr>
                                </thead>
                                <tbody>
                                    {victims.map(v => (
                                        <tr key={v.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                                            <td style={{ padding: '8px 4px' }}>{v.id.split('_')[1] ?? v.id}</td>
                                            <td style={{ padding: '8px 4px' }}>({v.world_x?.toFixed(1)}, {v.world_y?.toFixed(1)})</td>
                                            <td style={{ padding: '8px 4px', color: v.detection_status === 'DETECTED' ? 'var(--accent-green)' : 'var(--text-main)' }}>{v.detection_status}</td>
                                        </tr>
                                    ))}
                                    {victims.length === 0 && <tr><td colSpan={3} style={{ padding: '16px', textAlign: 'center', color: 'var(--text-muted)' }}>No victims recorded.</td></tr>}
                                </tbody>
                            </table>
                        </div>
                        <div className="panel flex-col" style={{ overflowY: 'auto' }}>
                            <div className="panel-title">DETECTIONS</div>
                            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem', marginTop: '12px' }}>
                                <thead>
                                    <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.2)', textAlign: 'left' }}>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Time</th>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Victim</th>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Drone</th>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Source</th>
                                        <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Distance</th>
                                    </tr>
                                </thead>
                                <tbody>
                                    {detections.map(d => (
                                        <tr key={d.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                                            <td style={{ padding: '8px 4px' }}>{d.timestamp?.toFixed(1)}</td>
                                            <td style={{ padding: '8px 4px' }}>{d.victim_id.split('_')[1] ?? d.victim_id}</td>
                                            <td style={{ padding: '8px 4px' }}>{d.drone_id}</td>
                                            <td style={{ padding: '8px 4px', fontWeight: 'bold' }}>{d.detection_source}</td>
                                            <td style={{ padding: '8px 4px' }}>{d.euclidean_distance?.toFixed(2)}m</td>
                                        </tr>
                                    ))}
                                    {detections.length === 0 && <tr><td colSpan={5} style={{ padding: '16px', textAlign: 'center', color: 'var(--text-muted)' }}>No detections recorded.</td></tr>}
                                </tbody>
                            </table>
                        </div>
                    </div>
                )}

                {activeTab === 'SAFETY' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">SAFETY EVENTS</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem', marginTop: '12px' }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.2)', textAlign: 'left' }}>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Time</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Drone(s)</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Conflict Type</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Action Taken</th>
                                    <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Details</th>
                                </tr>
                            </thead>
                            <tbody>
                                {safety.map(s => (
                                    <tr key={s.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                                        <td style={{ padding: '8px 4px' }}>{s.timestamp?.toFixed(1)}</td>
                                        <td style={{ padding: '8px 4px' }}>{s.drone_ids}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--text-warning)' }}>{s.conflict_type}</td>
                                        <td style={{ padding: '8px 4px' }}>{s.action_taken}</td>
                                        <td style={{ padding: '8px 4px' }}>{s.metadata}</td>
                                    </tr>
                                ))}
                                {safety.length === 0 && <tr><td colSpan={5} style={{ padding: '16px', textAlign: 'center', color: 'var(--text-muted)' }}>No safety events recorded.</td></tr>}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'TELEMETRY' && <TelemetryTable missionId={mission.id} />}
            </div>
        </div>
    );
}
