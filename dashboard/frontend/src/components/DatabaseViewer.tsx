import React, { useEffect, useState, useRef, useCallback } from 'react';
import { FinalEvaluationCard } from './FinalEvaluationCard';
import { formatCoverage } from '../utils/formatters';

// ─── Types ────────────────────────────────────────────────────────────────────

interface LiveSnapshot {
    active: boolean;
    state: string;
    mission_id?: string;
    map_id?: string;
    drone_count?: number;
    start_time?: number;
    elapsed?: number;
    coverage?: number;
    victims_detected?: number;
    total_victims?: number;
    decision_count?: number;
    max_decisions?: number;
    safety_overrides?: number;
    sim_rtf?: number | null;
    telemetry_age_sec?: number | null;
    drones?: LiveDrone[];
    victims?: LiveVictim[];
    tracked_victims?: LiveVictim[];
    tasks?: LiveTask[];
    task_allocation?: {
        total_tasks: number;
        active_tasks: number;
        completed_tasks: number;
        failed_tasks: number;
        state_counts: Record<string, number>;
        active_assignments?: Record<string, string>;
    };
    episodes_recorded?: number;
    avg_episode_coverage?: number;
    telemetry_rows?: number;
    detection_events?: number;
}

interface LiveTask {
    task_id: string;
    victim_id: string;
    world_x: number;
    world_y: number;
    state: string;
    assigned_drone_id?: string | null;
    confidence?: number;
    verification_mode?: string;
    reassignment_count?: number;
    investigation_progress_sec?: number;
    investigation_duration?: number;
}

interface SARTaskRecord {
    id: string;
    mission_id: string;
    victim_id: string;
    world_x: number;
    world_y: number;
    grid_x: number;
    grid_y: number;
    state: string;
    assigned_drone_id?: string | null;
    confidence?: number;
    verification_mode?: string;
    evidence_state?: string;
    created_sim_time?: number;
    updated_sim_time?: number;
    assigned_sim_time?: number | null;
    investigation_start_sim_time?: number | null;
    completed_sim_time?: number | null;
    failed_sim_time?: number | null;
    failure_reason?: string | null;
    reassignment_count?: number;
}

interface LiveDrone {
    id: string;
    state: string;
    action: string;
    x: number;
    y: number;
    z: number;
    safety_override: boolean;
}

interface LiveVictim {
    id: string;
    state: string;
    detected: boolean;
    world_x: number;
    world_y: number;
    detected_by?: string;
    confidence?: number;
    source?: string;
    gt_error?: number;
}

interface Experiment {
    id: string;
    type: string;
    name: string;
    timestamp: number;
    environment: string;
    qmix_checkpoint: string | null;
}

interface Mission {
    id: string;
    experiment_id: string;
    map_id: string;
    drone_count: number;
    victim_count: number;
    start_time: number;
    end_time: number | null;
    status: string;
    final_coverage: number | null;
    safety_overrides: number | null;
}

interface Episode {
    id: string;
    episode_number: number;
    duration: number;
    coverage: number;
    victims_found: number;
    timeout_count: number;
    invalid_flag: boolean;
    status: string;
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
    confidence: number;
}

interface SafetyEvent {
    id: number;
    timestamp: number;
    drone_ids: string;
    conflict_type: string;
    action_taken: string;
    metadata: string;
}

// ─── Formatting Helpers ────────────────────────────────────────────────────────

const fmtDuration = (sec: number) => {
    if (sec < 60) return `${sec.toFixed(0)}s`;
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return `${m}m ${s}s`;
};

const fmtTimestamp = (unix: number) =>
    new Date(unix * 1000).toLocaleString(undefined, { dateStyle: 'short', timeStyle: 'medium' });

const stateColor = (s: string) => {
    if (s === 'RUNNING') return 'var(--accent-green)';
    if (s === 'COMPLETE') return 'var(--accent-cyan)';
    if (s === 'QMIX_STARTING') return '#a78bfa';
    if (s === 'STARTING' || s === 'SIMULATOR_READY') return '#fbbf24';
    if (s === 'STOPPED' || s === 'STOPPING') return 'var(--text-muted)';
    if (s === 'ERROR') return 'var(--text-warning)';
    return 'var(--text-main)';
};

// ─── Pulse dot component ──────────────────────────────────────────────────────

function PulseDot({ color = 'var(--accent-green)' }: { color?: string }) {
    return (
        <span style={{ display: 'inline-block', position: 'relative', width: 10, height: 10, marginRight: 8, verticalAlign: 'middle' }}>
            <span style={{
                position: 'absolute', inset: 0, borderRadius: '50%',
                background: color, animation: 'pulse-ring 1.4s ease infinite', opacity: 0.5
            }} />
            <span style={{
                position: 'absolute', inset: 2, borderRadius: '50%',
                background: color
            }} />
        </span>
    );
}

// ─── Stat Box ──────────────────────────────────────────────────────────────────

function StatBox({ label, value, color, sub }: { label: string; value: React.ReactNode; color?: string; sub?: string }) {
    return (
        <div style={{
            background: 'rgba(255,255,255,0.03)', borderRadius: 6,
            border: '1px solid rgba(255,255,255,0.07)',
            padding: '10px 14px', display: 'flex', flexDirection: 'column', gap: 4
        }}>
            <div style={{ fontSize: '0.72rem', color: 'var(--text-muted)', letterSpacing: '0.06em', textTransform: 'uppercase' }}>{label}</div>
            <div style={{ fontSize: '1.35rem', fontWeight: 700, color: color || 'var(--text-main)', fontFamily: 'monospace' }}>{value}</div>
            {sub && <div style={{ fontSize: '0.72rem', color: 'var(--text-muted)' }}>{sub}</div>}
        </div>
    );
}

// ─── LIVE MISSION PANEL ───────────────────────────────────────────────────────

function LiveMissionPanel({ snap }: { snap: LiveSnapshot }) {
    const [elapsed, setElapsed] = useState(snap.elapsed ?? 0);

    useEffect(() => {
        // Drive elapsed locally for smooth ticking between polls
        const start = snap.start_time ? snap.start_time : (Date.now() / 1000 - (snap.elapsed ?? 0));
        const interval = setInterval(() => {
            setElapsed(Math.round(Date.now() / 1000 - start));
        }, 1000);
        return () => clearInterval(interval);
    }, [snap.start_time, snap.elapsed]);

    const drones = snap.drones ?? [];
    const victims = snap.victims ?? [];
    const detectedVictims = victims.filter(v => v.detected);
    const covPct = snap.coverage ?? 0;
    const maxDec = snap.max_decisions ?? 0;
    const curDec = snap.decision_count ?? 0;
    const decPct = maxDec > 0 ? Math.min(100, (curDec / maxDec) * 100) : 0;

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 16, flex: 1, minHeight: 0 }}>
            {/* Header Banner */}
            <div className="panel" style={{ padding: '14px 18px', borderColor: stateColor(snap.state) }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', flexWrap: 'wrap', gap: 10 }}>
                    <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
                        <PulseDot color={stateColor(snap.state)} />
                        <span style={{ fontWeight: 800, fontSize: '1rem', letterSpacing: '0.08em', color: stateColor(snap.state) }}>
                            LIVE MISSION — {snap.state}
                        </span>
                    </div>
                    <div style={{ display: 'flex', gap: 20, fontSize: '0.8rem', color: 'var(--text-muted)', fontFamily: 'monospace' }}>
                        <span>MAP: <strong style={{ color: 'var(--text-main)' }}>{snap.map_id?.toUpperCase()}</strong></span>
                        <span>ID: <strong style={{ color: 'var(--text-main)' }}>{snap.mission_id?.slice(0, 8)}</strong></span>
                        {snap.telemetry_age_sec !== null && snap.telemetry_age_sec !== undefined && (
                            <span>ROS: <strong style={{ color: snap.telemetry_age_sec > 3 ? 'var(--text-warning)' : 'var(--accent-green)' }}>
                                {snap.telemetry_age_sec.toFixed(1)}s ago
                            </strong></span>
                        )}
                    </div>
                </div>
            </div>

            {/* Main Metrics Grid */}
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 12 }}>
                <StatBox label="Elapsed" value={fmtDuration(elapsed)} color="var(--text-main)" />
                <StatBox label="Coverage" value={`${covPct.toFixed(1)}%`} color="var(--accent-cyan)"
                    sub={`${snap.decision_count ?? 0} / ${maxDec} decisions`} />
                <StatBox label="Victims Found" value={`${snap.victims_detected ?? 0} / ${snap.total_victims ?? 0}`}
                    color={snap.victims_detected === snap.total_victims && (snap.total_victims ?? 0) > 0 ? 'var(--accent-green)' : 'var(--text-main)'} />
                <StatBox label="Safety Overrides" value={snap.safety_overrides ?? 0}
                    color={(snap.safety_overrides ?? 0) > 0 ? 'var(--text-warning)' : 'var(--text-main)'} />
            </div>

            {/* Progress Bars */}
            <div className="panel" style={{ padding: '14px 18px' }}>
                <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
                    {/* Coverage */}
                    <div>
                        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 6, fontSize: '0.78rem', color: 'var(--text-muted)' }}>
                            <span>SEARCH COVERAGE</span>
                            <span style={{ color: 'var(--accent-cyan)', fontWeight: 700 }}>{covPct.toFixed(2)}%</span>
                        </div>
                        <div style={{ height: 10, background: 'rgba(255,255,255,0.05)', borderRadius: 5, overflow: 'hidden' }}>
                            <div style={{
                                height: '100%', width: `${Math.min(100, covPct)}%`,
                                background: 'linear-gradient(90deg, var(--accent-cyan), #00aaff)',
                                borderRadius: 5, transition: 'width 0.5s ease'
                            }} />
                        </div>
                    </div>
                    {/* Decisions */}
                    {maxDec > 0 && (
                        <div>
                            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 6, fontSize: '0.78rem', color: 'var(--text-muted)' }}>
                                <span>QMIX DECISIONS</span>
                                <span style={{ color: '#a78bfa', fontWeight: 700 }}>{curDec} / {maxDec}</span>
                            </div>
                            <div style={{ height: 10, background: 'rgba(255,255,255,0.05)', borderRadius: 5, overflow: 'hidden' }}>
                                <div style={{
                                    height: '100%', width: `${decPct}%`,
                                    background: 'linear-gradient(90deg, #7c3aed, #a78bfa)',
                                    borderRadius: 5, transition: 'width 0.5s ease'
                                }} />
                            </div>
                        </div>
                    )}
                </div>
            </div>

            {/* Drone States + Victim Grid */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16, flex: 1, minHeight: 0 }}>
                {/* Drone States */}
                <div className="panel flex-col" style={{ overflowY: 'auto' }}>
                    <div className="panel-title">DRONE AGENTS ({drones.length})</div>
                    <div style={{ display: 'flex', flexDirection: 'column', gap: 8, marginTop: 12 }}>
                        {drones.length === 0 ? (
                            <div className="text-muted" style={{ fontSize: '0.85rem', padding: 8 }}>
                                Awaiting telemetry…
                            </div>
                        ) : drones.map(d => (
                            <div key={d.id} style={{
                                display: 'flex', justifyContent: 'space-between', alignItems: 'center',
                                padding: '8px 12px',
                                background: d.safety_override ? 'rgba(255,100,0,0.08)' : 'rgba(255,255,255,0.02)',
                                border: `1px solid ${d.safety_override ? 'rgba(255,100,0,0.3)' : 'rgba(255,255,255,0.06)'}`,
                                borderRadius: 5
                            }}>
                                <div>
                                    <span style={{ fontWeight: 700, fontSize: '0.85rem', fontFamily: 'monospace' }}>
                                        {d.id.replace('drone_', 'D').toUpperCase()}
                                    </span>
                                    {d.safety_override && (
                                        <span style={{ marginLeft: 8, fontSize: '0.7rem', color: 'var(--text-warning)', fontWeight: 700 }}>⚠ OVERRIDE</span>
                                    )}
                                </div>
                                <div style={{ textAlign: 'right', fontSize: '0.8rem' }}>
                                    <div style={{ color: stateColor(d.state), fontWeight: 600 }}>{d.state}</div>
                                    <div className="text-muted" style={{ fontSize: '0.72rem' }}>
                                        ({d.x}, {d.y}) z={d.z}
                                    </div>
                                    <div style={{ color: '#a78bfa', fontSize: '0.72rem' }}>{d.action}</div>
                                </div>
                            </div>
                        ))}
                    </div>
                </div>

                {/* Victim Tracking */}
                <div className="panel flex-col" style={{ overflowY: 'auto' }}>
                    <div className="panel-title">
                        VICTIM TRACKING ({detectedVictims.length}/{victims.length})
                    </div>
                    <div style={{ display: 'flex', flexDirection: 'column', gap: 8, marginTop: 12 }}>
                        {victims.length === 0 ? (
                            <div className="text-muted" style={{ fontSize: '0.85rem', padding: 8 }}>
                                Awaiting telemetry…
                            </div>
                        ) : victims.map(v => (
                            <div key={v.id} style={{
                                display: 'flex', justifyContent: 'space-between', alignItems: 'center',
                                padding: '8px 12px',
                                background: v.detected ? 'rgba(0,255,136,0.05)' : 'rgba(255,255,255,0.02)',
                                border: `1px solid ${v.detected ? 'rgba(0,255,136,0.25)' : 'rgba(255,255,255,0.06)'}`,
                                borderRadius: 5
                            }}>
                                <div>
                                    <span style={{
                                        fontWeight: 700, fontSize: '0.85rem', fontFamily: 'monospace',
                                        color: v.detected ? 'var(--accent-green)' : 'var(--text-muted)'
                                    }}>
                                        {v.id.replace('victim_', 'V').toUpperCase()}
                                    </span>
                                    {v.detected && v.source && (
                                        <span style={{ marginLeft: 8, fontSize: '0.7rem', color: 'var(--accent-green)' }}>
                                            [{v.source}]
                                        </span>
                                    )}
                                </div>
                                <div style={{ textAlign: 'right', fontSize: '0.77rem' }}>
                                    <div style={{ color: v.detected ? 'var(--accent-green)' : 'var(--text-muted)', fontWeight: 600 }}>
                                        {v.detected ? '✓ DETECTED' : '○ SEARCHING'}
                                    </div>
                                    <div className="text-muted">({v.world_x}, {v.world_y})</div>
                                    {v.detected_by && (
                                        <div style={{ color: '#a78bfa', fontSize: '0.7rem' }}>
                                            by {v.detected_by.replace('drone_', 'D').toUpperCase()}
                                            {v.gt_error !== undefined && v.gt_error !== null && (
                                                <> · Δ{v.gt_error.toFixed(2)}m</>
                                            )}
                                        </div>
                                    )}
                                </div>
                            </div>
                        ))}
                    </div>
                </div>
            </div>

            {/* DB Write Stats */}
            <div className="panel" style={{ padding: '10px 18px' }}>
                <div style={{ display: 'flex', gap: 24, fontSize: '0.8rem', color: 'var(--text-muted)', flexWrap: 'wrap' }}>
                    <span>📊 DB TELEMETRY ROWS: <strong style={{ color: 'var(--text-main)' }}>{(snap.telemetry_rows ?? 0).toLocaleString()}</strong></span>
                    <span>🎯 DETECTION EVENTS: <strong style={{ color: 'var(--text-main)' }}>{snap.detection_events ?? 0}</strong></span>
                    {snap.tasks && snap.tasks.length > 0 && (
                        <span>🚁 SAR TASKS: <strong style={{ color: 'var(--accent-cyan)' }}>{snap.tasks.filter(t => t.state !== 'COMPLETED' && t.state !== 'FAILED').length} active / {snap.tasks.length} total</strong></span>
                    )}
                    <span>🔄 EPISODES RECORDED: <strong style={{ color: 'var(--text-main)' }}>{snap.episodes_recorded ?? 0}</strong></span>
                    {(snap.avg_episode_coverage ?? 0) > 0 && (
                        <span>📈 AVG EP COVERAGE: <strong style={{ color: 'var(--accent-cyan)' }}>{formatCoverage(snap.avg_episode_coverage, 1)}</strong></span>
                    )}
                    {snap.sim_rtf !== null && snap.sim_rtf !== undefined && (
                        <span>⚡ SIM RTF: <strong style={{ color: 'var(--accent-cyan)' }}>{snap.sim_rtf.toFixed(2)}x</strong></span>
                    )}
                </div>
            </div>
        </div>
    );
}

// ─── HISTORICAL MISSION DETAILS ────────────────────────────────────────────────

function TelemetryTable({ missionId }: { missionId: string }) {
    const [rows, setRows] = useState<any[]>([]);
    const [total, setTotal] = useState(0);
    const [offset, setOffset] = useState(0);
    const LIMIT = 100;

    useEffect(() => {
        setOffset(0);
    }, [missionId]);

    useEffect(() => {
        fetch(`/api/db/telemetry?mission_id=${missionId}&limit=${LIMIT}&offset=${offset}`)
            .then(r => r.json())
            .then(d => { setRows(d.items ?? []); setTotal(d.total ?? 0); })
            .catch(() => {});
    }, [missionId, offset]);

    return (
        <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto', minHeight: 0 }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <div className="panel-title">TELEMETRY ROWS (Total: {total.toLocaleString()})</div>
                <div style={{ display: 'flex', gap: 8 }}>
                    <button disabled={offset === 0} onClick={() => setOffset(Math.max(0, offset - LIMIT))}
                        style={{ background: 'transparent', border: '1px solid rgba(255,255,255,0.2)', color: 'white', padding: '3px 10px', borderRadius: 4, cursor: 'pointer' }}>◀</button>
                    <span className="text-muted" style={{ fontSize: '0.8rem', lineHeight: '28px' }}>{offset + 1}–{Math.min(offset + LIMIT, total)}</span>
                    <button disabled={offset + LIMIT >= total} onClick={() => setOffset(offset + LIMIT)}
                        style={{ background: 'transparent', border: '1px solid rgba(255,255,255,0.2)', color: 'white', padding: '3px 10px', borderRadius: 4, cursor: 'pointer' }}>▶</button>
                </div>
            </div>
            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.8rem', marginTop: 12 }}>
                <thead>
                    <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                        {['Time', 'Drone', 'X', 'Y', 'Z', 'State', 'Action'].map(h => (
                            <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                        ))}
                    </tr>
                </thead>
                <tbody>
                    {rows.map((r: any, i) => (
                        <tr key={i} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                            <td style={{ padding: '5px 4px', fontFamily: 'monospace', fontSize: '0.75rem' }}>{r.timestamp?.toFixed(1)}</td>
                            <td style={{ padding: '5px 4px', color: '#a78bfa' }}>{r.drone_id}</td>
                            <td style={{ padding: '5px 4px', fontFamily: 'monospace' }}>{r.x?.toFixed(1)}</td>
                            <td style={{ padding: '5px 4px', fontFamily: 'monospace' }}>{r.y?.toFixed(1)}</td>
                            <td style={{ padding: '5px 4px', fontFamily: 'monospace' }}>{r.z?.toFixed(1)}</td>
                            <td style={{ padding: '5px 4px', color: stateColor(r.state ?? '') }}>{r.state}</td>
                            <td style={{ padding: '5px 4px', color: 'var(--text-muted)' }}>{r.action}</td>
                        </tr>
                    ))}
                    {rows.length === 0 && (
                        <tr><td colSpan={7} style={{ padding: 16, textAlign: 'center', color: 'var(--text-muted)' }}>No telemetry rows.</td></tr>
                    )}
                </tbody>
            </table>
        </div>
    );
}

function MissionDetails({ missionId, onBack }: { missionId: string; onBack: () => void }) {
    const [mission, setMission] = useState<Mission | null>(null);
    const [episodes, setEpisodes] = useState<Episode[]>([]);
    const [victims, setVictims] = useState<Victim[]>([]);
    const [tasks, setTasks] = useState<SARTaskRecord[]>([]);
    const [detections, setDetections] = useState<Detection[]>([]);
    const [safety, setSafety] = useState<SafetyEvent[]>([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);
    const [activeTab, setActiveTab] = useState<'EPISODES' | 'VICTIMS' | 'TASKS' | 'DETECTIONS' | 'SAFETY' | 'TELEMETRY'>('EPISODES');

    useEffect(() => {
        setLoading(true);
        Promise.all([
            fetch(`/api/db/missions/${missionId}`).then(r => r.json()),
            fetch(`/api/db/episodes?mission_id=${missionId}`).then(r => r.json()),
            fetch(`/api/db/victims?mission_id=${missionId}`).then(r => r.json()),
            fetch(`/api/db/tasks?mission_id=${missionId}`).then(r => r.json()).catch(() => []),
            fetch(`/api/db/detections?mission_id=${missionId}`).then(r => r.json()),
            fetch(`/api/db/safety?mission_id=${missionId}`).then(r => r.json()),
        ]).then(([m, ep, v, t, d, s]) => {
            if (m.detail) throw new Error(m.detail);
            setMission(m); setEpisodes(ep); setVictims(v); setTasks(Array.isArray(t) ? t : []); setDetections(d); setSafety(s);
        }).catch(err => setError(err.message))
          .finally(() => setLoading(false));
    }, [missionId]);

    if (loading) return <div className="panel flex-col" style={{ flex: 1 }}>Loading mission details…</div>;
    if (error) return <div className="panel flex-col" style={{ flex: 1, color: 'var(--text-warning)' }}>Error: {error}</div>;
    if (!mission) return null;

    const tabs = ['EPISODES', 'VICTIMS', 'TASKS', 'DETECTIONS', 'SAFETY', 'TELEMETRY'] as const;

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 16, flex: 1, minHeight: 0 }}>
            {/* Summary Header */}
            <div className="panel" style={{ padding: '14px 18px' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 14 }}>
                    <div className="panel-title">MISSION DETAILS</div>
                    <button onClick={onBack} style={{
                        background: 'transparent', border: '1px solid rgba(255,255,255,0.2)',
                        color: 'white', padding: '4px 14px', borderRadius: 4, cursor: 'pointer', fontSize: '0.85rem'
                    }}>← Back</button>
                </div>
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(6, 1fr)', gap: 10 }}>
                    {[
                        { label: 'ID', value: mission.id.slice(0, 8), color: 'var(--text-main)' },
                        { label: 'Status', value: mission.status, color: stateColor(mission.status) },
                        { label: 'Map', value: mission.map_id },
                        { label: 'Drones / Victims', value: `${mission.drone_count} / ${mission.victim_count}` },
                        { label: 'Coverage', value: mission.final_coverage !== null && mission.final_coverage !== undefined ? formatCoverage(mission.final_coverage) : 'N/A', color: 'var(--accent-cyan)' },
                        { label: 'Duration', value: mission.end_time && mission.start_time ? fmtDuration(mission.end_time - mission.start_time) : 'N/A' },
                    ].map(({ label, value, color }) => (
                        <div key={label}>
                            <div style={{ fontSize: '0.72rem', color: 'var(--text-muted)', marginBottom: 3 }}>{label}</div>
                            <div style={{ fontWeight: 600, color: color || 'var(--text-main)' }}>{value}</div>
                        </div>
                    ))}
                </div>
            </div>

            {/* Tabs */}
            <div style={{ display: 'flex', gap: 6 }}>
                {tabs.map(t => (
                    <button key={t} onClick={() => setActiveTab(t)} style={{
                        background: activeTab === t ? 'var(--accent-cyan)' : 'var(--panel-bg)',
                        color: activeTab === t ? 'black' : 'var(--text-muted)',
                        border: `1px solid ${activeTab === t ? 'var(--accent-cyan)' : 'rgba(255,255,255,0.1)'}`,
                        padding: '5px 16px', borderRadius: 4, cursor: 'pointer', fontWeight: 700, fontSize: '0.82rem'
                    }}>{t}</button>
                ))}
            </div>

            {/* Tab Content */}
            <div style={{ flex: 1, display: 'flex', flexDirection: 'column', minHeight: 0 }}>
                {activeTab === 'EPISODES' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">EPISODES ({episodes.length})</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.82rem', marginTop: 12 }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                                    {['Ep #', 'Duration', 'Coverage', 'Victims', 'Timeouts', 'Invalid', 'Status'].map(h => (
                                        <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                                    ))}
                                </tr>
                            </thead>
                            <tbody>
                                {episodes.map(e => (
                                    <tr key={e.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace', fontWeight: 700 }}>{e.episode_number ?? 'N/A'}</td>
                                        <td style={{ padding: '8px 4px' }}>{fmtDuration(e.duration ?? 0)}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--accent-cyan)', fontWeight: 700 }}>{e.coverage !== null && e.coverage !== undefined ? formatCoverage(e.coverage) : 'N/A'}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--accent-green)', fontWeight: 600 }}>{e.victims_found ?? 0}</td>
                                        <td style={{ padding: '8px 4px' }}>{e.timeout_count ?? 0}</td>
                                        <td style={{ padding: '8px 4px', color: e.invalid_flag ? 'var(--text-warning)' : 'var(--text-muted)' }}>
                                            {e.invalid_flag ? '⚠ Yes' : 'No'}
                                        </td>
                                        <td style={{ padding: '8px 4px', color: stateColor(e.status ?? '') }}>{e.status}</td>
                                    </tr>
                                ))}
                                {episodes.length === 0 && (
                                    <tr><td colSpan={7} style={{ padding: 16, textAlign: 'center', color: 'var(--text-muted)' }}>No episodes recorded.</td></tr>
                                )}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'VICTIMS' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">VICTIMS ({victims.length})</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.82rem', marginTop: 12 }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                                    {['ID', 'Grid (X,Y)', 'World (X,Y)', 'Status'].map(h => (
                                        <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                                    ))}
                                </tr>
                            </thead>
                            <tbody>
                                {victims.map(v => (
                                    <tr key={v.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace', fontWeight: 700 }}>{v.id.split('_')[1] ?? v.id}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>({v.grid_x},{v.grid_y})</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>({v.world_x?.toFixed(1)}, {v.world_y?.toFixed(1)})</td>
                                        <td style={{ padding: '8px 4px', color: v.detection_status === 'DETECTED' ? 'var(--accent-green)' : 'var(--text-muted)', fontWeight: 600 }}>
                                            {v.detection_status}
                                        </td>
                                    </tr>
                                ))}
                                {victims.length === 0 && (
                                    <tr><td colSpan={4} style={{ padding: 16, textAlign: 'center', color: 'var(--text-muted)' }}>No victims recorded.</td></tr>
                                )}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'TASKS' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">AUTONOMOUS SAR TASKS ({tasks.length})</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.82rem', marginTop: 12 }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                                    {['Task ID', 'Victim', 'Target (X, Y)', 'State', 'Drone', 'Conf', 'Mode', 'Reassigns', 'Created', 'Completed', 'Failure'].map(h => (
                                        <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                                    ))}
                                </tr>
                            </thead>
                            <tbody>
                                {tasks.map(t => (
                                    <tr key={t.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace', fontWeight: 700 }}>{t.id}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{t.victim_id}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>({t.world_x?.toFixed(1)}, {t.world_y?.toFixed(1)})</td>
                                        <td style={{ padding: '8px 4px', fontWeight: 600, color: t.state === 'COMPLETED' ? 'var(--accent-green)' : t.state === 'FAILED' ? 'var(--text-warning)' : 'var(--accent-cyan)' }}>
                                            {t.state}
                                        </td>
                                        <td style={{ padding: '8px 4px', color: '#a78bfa' }}>{t.assigned_drone_id ? t.assigned_drone_id.replace('drone_', 'D').toUpperCase() : 'None'}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>
                                            {t.confidence !== null && t.confidence !== undefined ? `${(t.confidence * 100).toFixed(0)}%` : 'N/A'}
                                        </td>
                                        <td style={{ padding: '8px 4px', color: 'var(--text-muted)', fontSize: '0.75rem' }}>{t.verification_mode || 'RGBD_THERMAL'}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{t.reassignment_count ?? 0}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{t.created_sim_time !== null && t.created_sim_time !== undefined ? `${t.created_sim_time.toFixed(1)}s` : 'N/A'}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{t.completed_sim_time !== null && t.completed_sim_time !== undefined ? `${t.completed_sim_time.toFixed(1)}s` : 'N/A'}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--text-warning)', fontSize: '0.75rem' }}>{t.failure_reason || '-'}</td>
                                    </tr>
                                ))}
                                {tasks.length === 0 && (
                                    <tr><td colSpan={11} style={{ padding: 16, textAlign: 'center', color: 'var(--text-muted)' }}>No SAR tasks recorded for this mission.</td></tr>
                                )}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'DETECTIONS' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">DETECTION EVENTS ({detections.length})</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.82rem', marginTop: 12 }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                                    {['Time', 'Victim', 'Drone', 'Source', 'Distance', 'Confidence'].map(h => (
                                        <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                                    ))}
                                </tr>
                            </thead>
                            <tbody>
                                {detections.map(d => (
                                    <tr key={d.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{d.timestamp?.toFixed(1)}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace', fontWeight: 700 }}>{d.victim_id.split('_')[1] ?? d.victim_id}</td>
                                        <td style={{ padding: '8px 4px', color: '#a78bfa' }}>{d.drone_id}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--accent-cyan)', fontWeight: 700 }}>{d.detection_source}</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{d.euclidean_distance?.toFixed(2)}m</td>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>
                                            {d.confidence !== null && d.confidence !== undefined ? (d.confidence * 100).toFixed(0) + '%' : 'N/A'}
                                        </td>
                                    </tr>
                                ))}
                                {detections.length === 0 && (
                                    <tr><td colSpan={6} style={{ padding: 16, textAlign: 'center', color: 'var(--text-muted)' }}>No detection events.</td></tr>
                                )}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'SAFETY' && (
                    <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                        <div className="panel-title">SAFETY EVENTS ({safety.length})</div>
                        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.82rem', marginTop: 12 }}>
                            <thead>
                                <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                                    {['Time', 'Drone(s)', 'Conflict Type', 'Action Taken', 'Metadata'].map(h => (
                                        <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                                    ))}
                                </tr>
                            </thead>
                            <tbody>
                                {safety.map(s => (
                                    <tr key={s.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                                        <td style={{ padding: '8px 4px', fontFamily: 'monospace' }}>{s.timestamp?.toFixed(1)}</td>
                                        <td style={{ padding: '8px 4px', color: '#a78bfa' }}>{s.drone_ids}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--text-warning)', fontWeight: 700 }}>{s.conflict_type}</td>
                                        <td style={{ padding: '8px 4px' }}>{s.action_taken}</td>
                                        <td style={{ padding: '8px 4px', color: 'var(--text-muted)', fontSize: '0.75rem' }}>{s.metadata}</td>
                                    </tr>
                                ))}
                                {safety.length === 0 && (
                                    <tr><td colSpan={5} style={{ padding: 16, textAlign: 'center', color: 'var(--text-muted)' }}>No safety events.</td></tr>
                                )}
                            </tbody>
                        </table>
                    </div>
                )}

                {activeTab === 'TELEMETRY' && <TelemetryTable missionId={missionId} />}
            </div>
        </div>
    );
}

// ─── HISTORICAL EXPERIMENT LIST ────────────────────────────────────────────────

function HistoricalView({ onSelectMission }: { onSelectMission: (id: string) => void }) {
    const [experiments, setExperiments] = useState<Experiment[]>([]);
    const [missions, setMissions] = useState<Mission[]>([]);
    const [selectedExpId, setSelectedExpId] = useState<string | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        fetch('/api/db/experiments')
            .then(r => {
                if (!r.ok) throw new Error(`Server returned status ${r.status}`);
                return r.json();
            })
            .then(data => {
                if (Array.isArray(data)) {
                    setExperiments(data);
                    if (data.length > 0) setSelectedExpId(data[0].id);
                } else {
                    setExperiments([]);
                }
            })
            .catch(err => {
                console.error("Failed to fetch experiments:", err);
                setError(err.message);
            })
            .finally(() => setLoading(false));
    }, []);

    useEffect(() => {
        if (!selectedExpId) return;
        fetch(`/api/db/missions?experiment_id=${selectedExpId}`)
            .then(r => {
                if (!r.ok) throw new Error(`Server returned status ${r.status}`);
                return r.json();
            })
            .then(data => {
                if (Array.isArray(data)) setMissions(data);
                else setMissions([]);
            })
            .catch(console.error);
    }, [selectedExpId]);

    if (loading) return <div className="panel">Loading experiments…</div>;
    if (error) return <div className="panel" style={{ color: 'var(--text-warning)' }}>Error: {error}</div>;

    const selectedExp = experiments.find(e => e.id === selectedExpId);
    const sortedMissions = [...missions].sort((a, b) => (b.start_time ?? 0) - (a.start_time ?? 0));

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 16, flex: 1, minHeight: 0 }}>
            {/* Evaluation + Notes */}
            <div style={{ display: 'flex', gap: 16, height: 160 }}>
                <FinalEvaluationCard />
                <div className="panel flex-col" style={{ flex: 1, justifyContent: 'center', alignItems: 'center' }}>
                    <div className="panel-title" style={{ marginBottom: 8 }}>SWARM DEPLOYMENT SPECIFICATIONS</div>
                    <div className="text-muted" style={{ fontSize: '0.85rem', textAlign: 'center', maxWidth: '80%' }}>
                        Multi-agent aerial swarm coordination operational across 1 to 6 autonomous UAVs.<br />
                        Decentralized QMIX neural policy with deterministic APF safety guarantees.<br />
                        Mission telemetry and victim discovery records persisted to SQLite.
                    </div>
                </div>
            </div>

            {/* Experiment List + Mission Table */}
            <div style={{ display: 'flex', gap: 16, flex: 1, minHeight: 0 }}>
                {/* Sidebar */}
                <div className="panel flex-col" style={{ width: 320, overflowY: 'auto' }}>
                    <div className="panel-title">EXPERIMENTS ({experiments.length})</div>
                    <div style={{ display: 'flex', flexDirection: 'column', gap: 8, marginTop: 12 }}>
                        {experiments.length === 0 ? (
                            <div className="text-muted" style={{ fontSize: '0.85rem', padding: 8 }}>No experiments recorded.</div>
                        ) : experiments.map(exp => (
                            <div key={exp.id} onClick={() => setSelectedExpId(exp.id)} style={{
                                padding: '10px 12px',
                                border: `1px solid ${selectedExpId === exp.id ? 'var(--accent-cyan)' : 'rgba(255,255,255,0.08)'}`,
                                borderRadius: 5, cursor: 'pointer',
                                background: selectedExpId === exp.id ? 'rgba(0,255,255,0.05)' : 'transparent',
                                transition: 'all 0.15s ease'
                            }}>
                                <div style={{ fontWeight: 700, fontSize: '0.88rem' }}>{exp.name || exp.id.slice(0, 8)}</div>
                                <div className="text-muted" style={{ fontSize: '0.77rem', marginTop: 3 }}>Type: {exp.type || 'N/A'}</div>
                                <div className="text-muted" style={{ fontSize: '0.75rem' }}>{fmtTimestamp(exp.timestamp)}</div>
                            </div>
                        ))}
                    </div>
                </div>

                {/* Mission Table */}
                <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                    <div className="panel-title">
                        MISSIONS {selectedExp ? `— ${selectedExp.name}` : ''}
                    </div>
                    {selectedExp && (
                        <div className="text-muted" style={{ fontSize: '0.82rem', marginBottom: 14, marginTop: 6 }}>
                            Checkpoint: {selectedExp.qmix_checkpoint || 'N/A'} · Env: {selectedExp.environment}
                        </div>
                    )}
                    <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem' }}>
                        <thead>
                            <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.15)', textAlign: 'left' }}>
                                {['Time', 'Status', 'Map', 'Drones', 'Victims', 'Coverage', 'Duration', ''].map(h => (
                                    <th key={h} style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>{h}</th>
                                ))}
                            </tr>
                        </thead>
                        <tbody>
                            {sortedMissions.map(m => (
                                <tr key={m.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.04)' }}>
                                    <td style={{ padding: '10px 4px', fontSize: '0.75rem', color: 'var(--text-muted)' }}>
                                        {m.start_time ? fmtTimestamp(m.start_time) : 'N/A'}
                                    </td>
                                    <td style={{ padding: '10px 4px', color: stateColor(m.status), fontWeight: 600 }}>{m.status}</td>
                                    <td style={{ padding: '10px 4px' }}>{m.map_id}</td>
                                    <td style={{ padding: '10px 4px', textAlign: 'center' }}>{m.drone_count}</td>
                                    <td style={{ padding: '10px 4px', textAlign: 'center' }}>{m.victim_count}</td>
                                    <td style={{ padding: '10px 4px', color: 'var(--accent-cyan)', fontWeight: 700 }}>
                                        {m.final_coverage !== null && m.final_coverage !== undefined ? formatCoverage(m.final_coverage) : 'N/A'}
                                    </td>
                                    <td style={{ padding: '10px 4px', fontFamily: 'monospace', fontSize: '0.8rem' }}>
                                        {m.end_time && m.start_time ? fmtDuration(m.end_time - m.start_time) : 'N/A'}
                                    </td>
                                    <td style={{ padding: '10px 4px' }}>
                                        <button onClick={() => onSelectMission(m.id)} style={{
                                            background: 'transparent', border: '1px solid var(--accent-cyan)',
                                            color: 'var(--accent-cyan)', padding: '3px 12px', borderRadius: 4, cursor: 'pointer',
                                            fontSize: '0.78rem', fontWeight: 700
                                        }}>Details →</button>
                                    </td>
                                </tr>
                            ))}
                            {sortedMissions.length === 0 && (
                                <tr><td colSpan={8} style={{ padding: 20, textAlign: 'center', color: 'var(--text-muted)' }}>
                                    No missions recorded for this experiment.
                                </td></tr>
                            )}
                        </tbody>
                    </table>
                </div>
            </div>
        </div>
    );
}

// ─── ROOT COMPONENT ────────────────────────────────────────────────────────────

export function DatabaseViewer() {
    const [selectedMissionId, setSelectedMissionId] = useState<string | null>(null);
    const [liveSnap, setLiveSnap] = useState<LiveSnapshot | null>(null);
    const [isLive, setIsLive] = useState(false);
    const [activeTab, setActiveTab] = useState<'LIVE' | 'DATABASE'>('LIVE');
    const pollRef = useRef<ReturnType<typeof setInterval> | null>(null);

    const pollLive = useCallback(() => {
        fetch('/api/db/live')
            .then(r => {
                if (!r.ok) return { active: false, state: 'IDLE' };
                return r.json();
            })
            .then((snap: LiveSnapshot) => {
                setLiveSnap(snap);
                setIsLive(snap.active === true);
            })
            .catch(() => setIsLive(false));
    }, []);

    useEffect(() => {
        pollLive();
        pollRef.current = setInterval(pollLive, 1500);
        return () => {
            if (pollRef.current) clearInterval(pollRef.current);
        };
    }, [pollLive]);

    // When a live mission ends, clear any selected mission detail so we fall back to history
    const prevIsLive = useRef(false);
    useEffect(() => {
        if (prevIsLive.current && !isLive) {
            setSelectedMissionId(null);
        }
        prevIsLive.current = isLive;
    }, [isLive]);

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 16, flex: 1, minHeight: 0, padding: 16 }}>
            {/* Pulse CSS injected once */}
            <style>{`
                @keyframes pulse-ring {
                    0% { transform: scale(1); opacity: 0.7; }
                    70% { transform: scale(2.2); opacity: 0; }
                    100% { transform: scale(1); opacity: 0; }
                }
            `}</style>

            {/* Mode Tabs Banner */}
            <div style={{
                display: 'flex', alignItems: 'center', justifyContent: 'space-between', flexWrap: 'wrap', gap: 12,
                padding: '8px 14px',
                background: 'rgba(255,255,255,0.03)',
                border: '1px solid rgba(255,255,255,0.08)',
                borderRadius: 8
            }}>
                <div style={{ display: 'flex', gap: 8 }}>
                    <button
                        onClick={() => setActiveTab('LIVE')}
                        style={{
                            padding: '6px 14px',
                            borderRadius: '6px',
                            border: activeTab === 'LIVE' ? '1px solid var(--accent-green)' : '1px solid transparent',
                            background: activeTab === 'LIVE' ? 'rgba(0,255,136,0.12)' : 'rgba(255,255,255,0.04)',
                            color: activeTab === 'LIVE' ? 'var(--accent-green)' : 'var(--text-muted)',
                            cursor: 'pointer',
                            fontWeight: 700,
                            fontSize: '0.8rem',
                            display: 'flex',
                            alignItems: 'center',
                            gap: '6px',
                            transition: 'all 0.15s'
                        }}
                    >
                        {isLive && <PulseDot color="var(--accent-green)" />}
                        ACTIVE MISSION {isLive ? '(LIVE)' : '(IDLE)'}
                    </button>

                    <button
                        onClick={() => setActiveTab('DATABASE')}
                        style={{
                            padding: '6px 14px',
                            borderRadius: '6px',
                            border: activeTab === 'DATABASE' ? '1px solid var(--accent-cyan)' : '1px solid transparent',
                            background: activeTab === 'DATABASE' ? 'rgba(0,242,254,0.12)' : 'rgba(255,255,255,0.04)',
                            color: activeTab === 'DATABASE' ? 'var(--accent-cyan)' : 'var(--text-muted)',
                            cursor: 'pointer',
                            fontWeight: 700,
                            fontSize: '0.8rem',
                            display: 'flex',
                            alignItems: 'center',
                            gap: '6px',
                            transition: 'all 0.15s'
                        }}
                    >
                        HISTORICAL EXPERIMENTS & DATABASE
                    </button>
                </div>

                <div className="text-muted" style={{ fontSize: '0.78rem' }}>
                    {activeTab === 'LIVE' && isLive && (
                        <span>State: <strong style={{ color: 'var(--accent-green)' }}>{liveSnap?.state}</strong> · Map: <strong>{liveSnap?.map_id}</strong> · Drones: <strong>{liveSnap?.drone_count}</strong></span>
                    )}
                    {activeTab === 'DATABASE' && (
                        <span>SQLite Persistent Storage: <code>antigravity.db</code></span>
                    )}
                </div>
            </div>

            {/* Main Content */}
            {activeTab === 'LIVE' && isLive && liveSnap ? (
                <LiveMissionPanel snap={liveSnap} />
            ) : selectedMissionId ? (
                <MissionDetails missionId={selectedMissionId} onBack={() => setSelectedMissionId(null)} />
            ) : (
                <HistoricalView onSelectMission={setSelectedMissionId} />
            )}
        </div>
    );
}
