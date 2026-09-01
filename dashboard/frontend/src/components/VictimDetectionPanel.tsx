import type { VictimState } from '../types/telemetry';

export function VictimDetectionPanel({ victims }: { victims: VictimState[] }) {
    if (!victims || victims.length === 0) {
        return (
            <div className="panel" style={{ marginTop: '1rem' }}>
                <h3>VICTIM DETECTION</h3>
                <div style={{ padding: '1rem', textAlign: 'center', color: 'var(--text-muted)' }}>
                    No victims currently tracked by visual perception.
                </div>
            </div>
        );
    }

    return (
        <div className="panel" style={{ marginTop: '1rem' }}>
            <h3>VICTIM DETECTION</h3>
            <div className="flex-col gap-2">
                {victims.map((v) => (
                    <div key={v.id} style={{
                        padding: '0.75rem',
                        background: 'rgba(255,255,255,0.03)',
                        borderRadius: '0.5rem',
                        borderLeft: `4px solid var(--accent-green)`
                    }}>
                        <div className="flex-row justify-between items-center" style={{ marginBottom: '0.5rem' }}>
                            <strong style={{ fontSize: '1.1rem', color: 'var(--text-main)', textTransform: 'uppercase' }}>
                                {v.id.replace('_', ' #')}
                            </strong>
                        </div>

                        <div style={{ display: 'grid', gridTemplateColumns: '1fr', gap: '0.4rem', fontSize: '0.85rem', color: 'var(--text-muted)' }}>
                            <div>
                                <span>Status: </span>
                                <strong style={{ color: 'var(--accent-green)' }}>{v.state || "DETECTED"}</strong>
                            </div>
                            <div>
                                <span>Last detected by: </span>
                                <strong style={{ color: 'var(--text-main)' }}>Drone {v.detected_by !== undefined ? v.detected_by : '?'}</strong>
                            </div>
                            <div>
                                <span>Observations: </span>
                                <strong style={{ color: 'var(--text-main)' }}>{v.observations !== undefined ? v.observations : 1}</strong>
                            </div>
                            <div>
                                <span>Last seen: </span>
                                <strong style={{ color: 'var(--text-main)' }}>{v.last_seen_sec_ago !== undefined ? `${v.last_seen_sec_ago.toFixed(1)} s ago` : 'N/A'}</strong>
                            </div>
                            <div>
                                <span>Position: </span>
                                <strong style={{ color: 'var(--text-main)' }}>
                                    {v.world_x !== undefined && v.world_y !== undefined
                                        ? `(${v.world_x.toFixed(1)}, ${v.world_y.toFixed(1)})`
                                        : `Grid (${v.x}, ${v.y})`}
                                </strong>
                            </div>
                        </div>
                    </div>
                ))}
            </div>
        </div>
    );
}
