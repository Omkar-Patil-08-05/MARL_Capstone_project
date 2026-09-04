import React, { useEffect, useState } from 'react';
import type { TelemetryPayload, BackendMissionStatus } from '../types/telemetry';

interface MissionSummaryProps {
    telemetry: TelemetryPayload | null;
    status: BackendMissionStatus;
    onClose: () => void;
}

export function MissionSummary({ telemetry, status, onClose }: MissionSummaryProps) {


    // Keep track of mission duration internally, or just display the max timestamp.
    // If backend provides a final timestamp in the status or if we tracked start, we could be exact.
    // Assuming telemetry.mission.decision_count * 0.2s or we can just fetch the /api/results on complete.

    const [finalDuration, setFinalDuration] = useState<number | null>(null);

    useEffect(() => {
        if (status.state === 'COMPLETE') {
            fetch('http://localhost:8000/api/results')
                .then(res => res.json())
                .then(data => {
                    const latest = data[data.length - 1];
                    if (latest) setFinalDuration(latest.mission_duration);
                })
                .catch(err => console.error(err));
        }
    }, [status.state]);

    if (status.state !== 'COMPLETE' || !telemetry) {
        return null;
    }

    const formatTime = (sec: number) => {
        const m = Math.floor(sec / 60);
        const s = Math.floor(sec % 60);
        return `${m}:${s.toString().padStart(2, '0')}`;
    };

    const coord = telemetry.coordination;
    const droneCount = coord ? coord.qmix_drones + coord.coord_drones : telemetry.drones.length;

    // QMIX agents are D0, D1... (based on qmix_drones)
    const qmixAgents = coord ? Array.from({length: coord.qmix_drones}, (_, i) => `D${i}`).join(', ') : 'None';
    // Coordinated Explorers are D(qmix_drones) to D(qmix_drones+coord_drones-1)
    const coordAgents = coord && coord.coord_drones > 0
        ? Array.from({length: coord.coord_drones}, (_, i) => `D${coord.qmix_drones + i}`).join(', ')
        : 'None';

    const totalObservations = (telemetry.tracked_victims || []).reduce((acc, v) => acc + (v.observations || 1), 0);

    return (
        <div style={{
            position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
            background: 'rgba(0,0,0,0.85)', zIndex: 9999,
            display: 'flex', alignItems: 'center', justifyContent: 'center'
        }}>
            <div className="panel flex-col" style={{ width: '400px', background: 'rgba(20,24,32,0.95)', border: '1px solid var(--accent-cyan)' }}>
                <h2 style={{ textAlign: 'center', color: 'var(--text-main)', marginBottom: '1.5rem', letterSpacing: '2px' }}>MISSION SUMMARY</h2>

                <div style={{ display: 'grid', gridTemplateColumns: '1fr', gap: '0.75rem', fontSize: '0.95rem' }}>
                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Drone Configuration</span>
                        <span style={{ fontWeight: 'bold' }}>{droneCount} Drones</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>QMIX Agents</span>
                        <span style={{ fontWeight: 'bold', color: 'var(--accent-blue)' }}>{qmixAgents}</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Coordinated Explorers</span>
                        <span style={{ fontWeight: 'bold', color: 'var(--accent-purple)' }}>{coordAgents}</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Mission Duration</span>
                        <span style={{ fontWeight: 'bold' }}>{finalDuration !== null ? formatTime(finalDuration) : '--:--'}</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Final Coverage</span>
                        <span style={{ fontWeight: 'bold', color: 'var(--accent-cyan)' }}>{telemetry.mission.coverage.toFixed(1)}%</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Victims Detected</span>
                        <span style={{ fontWeight: 'bold', color: 'var(--accent-green)' }}>{telemetry.mission.victims_detected} / {telemetry.mission.total_victims}</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between', borderBottom: '1px solid rgba(255,255,255,0.05)', paddingBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Victim Observations</span>
                        <span style={{ fontWeight: 'bold' }}>{totalObservations}</span>
                    </div>

                    <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Safety Interventions</span>
                        <span style={{ fontWeight: 'bold', color: 'var(--text-warning)' }}>{telemetry.mission.safety_overrides}</span>
                    </div>
                </div>

                <button
                    onClick={onClose}
                    className="btn"
                    style={{ marginTop: '2rem', width: '100%', padding: '12px' }}
                >
                    CLOSE SUMMARY
                </button>
            </div>
        </div>
    );
}
