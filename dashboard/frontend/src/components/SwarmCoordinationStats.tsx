import React from 'react';
import type { TelemetryPayload } from '../types/telemetry';

export function SwarmCoordinationStats({ telemetry }: { telemetry: TelemetryPayload | null }) {
    if (!telemetry || !telemetry.coordination) {
        return (
            <div className="panel" style={{ marginTop: '1rem' }}>
                <h3>SWARM COORDINATION</h3>
                <div style={{ padding: '1rem', textAlign: 'center', color: 'var(--text-muted)' }}>
                    Coordination data unavailable.
                </div>
            </div>
        );
    }

    const coord = telemetry.coordination;
    const activeFrontierCount = Object.keys(coord.active_frontiers || {}).length;
    const currentHolds = Object.keys(coord.safety_holds || {}).length;
    const totalInterventions = telemetry.mission.safety_overrides;

    return (
        <div className="panel" style={{ marginTop: '1rem' }}>
            <h3>SWARM COORDINATION</h3>

            <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', fontSize: '0.9rem' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Active drones:</span>
                    <strong style={{ color: 'var(--text-main)' }}>{coord.qmix_drones + coord.coord_drones}</strong>
                </div>

                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', padding: '0.5rem', background: 'rgba(255,255,255,0.03)', borderRadius: '4px' }}>
                    <div>
                        <span style={{ color: 'var(--text-muted)', display: 'block', marginBottom: '2px' }}>QMIX agents:</span>
                        {Array.from({length: coord.qmix_drones}).map((_, i) => (
                            <strong key={`q-${i}`} style={{ color: 'var(--accent-blue)', display: 'block' }}>D{i}</strong>
                        ))}
                    </div>
                    <div>
                        <span style={{ color: 'var(--text-muted)', display: 'block', marginBottom: '2px' }}>Coordinated explorers:</span>
                        {Array.from({length: coord.coord_drones}).map((_, i) => (
                            <strong key={`c-${i}`} style={{ color: 'var(--accent-purple)', display: 'block' }}>D{coord.qmix_drones + i}</strong>
                        ))}
                    </div>
                </div>

                <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Active frontier reservations:</span>
                    <strong style={{ color: 'var(--accent-green)' }}>{activeFrontierCount}</strong>
                </div>

                {activeFrontierCount > 0 && (
                    <div style={{ paddingLeft: '0.5rem', borderLeft: '2px solid rgba(255,255,255,0.1)' }}>
                        {Object.entries(coord.active_frontiers).map(([d_id, [gx, gy]]) => (
                            <div key={d_id} style={{ fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                                {d_id.replace('drone_', 'D')} → frontier ({gx}, {gy})
                            </div>
                        ))}
                    </div>
                )}

                <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Safety interventions:</span>
                    <strong style={{ color: 'var(--text-warning)' }}>{totalInterventions}</strong>
                </div>

                <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Current HOLD drones:</span>
                    <strong style={{ color: currentHolds > 0 ? 'var(--text-warning)' : 'var(--text-main)' }}>{currentHolds}</strong>
                </div>

                <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Coverage:</span>
                    <strong style={{ color: 'var(--accent-cyan)' }}>{telemetry.mission.coverage}%</strong>
                </div>
            </div>
        </div>
    );
}
