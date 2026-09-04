import type { VictimState } from '../types/telemetry';

interface VictimDetectionPanelProps {
    victims: VictimState[];
}

export function VictimDetectionPanel({ victims }: VictimDetectionPanelProps) {
    const detectedVictims = (victims || []).filter(v => v.detected);
    const totalVictims = (victims || []).length;

    if (totalVictims === 0) {
        return (
            <div className="panel" style={{ marginTop: '1rem' }}>
                <h3>VICTIM DETECTION</h3>
                <div style={{ padding: '1rem', textAlign: 'center', color: 'var(--text-muted)' }}>
                    Waiting for mission data...
                </div>
            </div>
        );
    }

    if (detectedVictims.length === 0) {
        return (
            <div className="panel" style={{ marginTop: '1rem' }}>
                <h3>VICTIM DETECTION <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', fontWeight: 400 }}>({totalVictims} total)</span></h3>
                <div style={{ padding: '1rem', textAlign: 'center', color: 'var(--text-muted)' }}>
                    No mission victims detected yet.
                </div>
            </div>
        );
    }

    return (
        <div className="panel" style={{ marginTop: '1rem' }}>
            <h3>VICTIM DETECTION <span style={{ fontSize: '0.8rem', color: 'var(--accent-green)', fontWeight: 400 }}>({detectedVictims.length}/{totalVictims})</span></h3>
            <div className="flex-col gap-2" style={{ maxHeight: '380px', overflowY: 'auto', paddingRight: '4px' }}>
                {detectedVictims.map((v) => (
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
                            <span style={{
                                fontSize: '0.7rem',
                                padding: '2px 8px',
                                borderRadius: '4px',
                                background: 'rgba(0, 255, 136, 0.15)',
                                color: 'var(--accent-green)',
                                fontWeight: 600,
                            }}>DETECTED</span>
                        </div>

                        <div style={{ display: 'grid', gridTemplateColumns: '1fr', gap: '0.4rem', fontSize: '0.85rem', color: 'var(--text-muted)' }}>
                            {v.detected_by !== undefined && v.detected_by !== null && (
                                <div>
                                    <span>Detected by: </span>
                                    <strong style={{ color: 'var(--text-main)' }}>
                                        {String(v.detected_by).replace('_', ' ').replace(/\b\w/g, l => l.toUpperCase())}
                                    </strong>
                                </div>
                            )}
                            {v.detection_distance !== undefined && v.detection_distance !== null && (
                                <div>
                                    <span>Distance at detection: </span>
                                    <strong style={{ color: 'var(--text-main)' }}>{v.detection_distance.toFixed(2)} m</strong>
                                </div>
                            )}
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
