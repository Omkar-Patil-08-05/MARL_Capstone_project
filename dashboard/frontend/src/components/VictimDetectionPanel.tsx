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
                            <strong style={{ fontSize: '1.1rem', color: 'var(--text-main)' }}>{v.id}</strong>
                            <span style={{
                                padding: '2px 6px',
                                borderRadius: '4px',
                                fontSize: '0.75rem',
                                fontWeight: 'bold',
                                background: 'rgba(0,255,100,0.1)',
                                color: 'var(--accent-green)'
                            }}>
                                STATUS: {v.state || "DETECTED"}
                            </span>
                        </div>
                        
                        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', fontSize: '0.85rem', color: 'var(--text-muted)' }}>
                            <div>
                                <span>Detection: </span>
                                <strong style={{ color: v.source === 'YOLO' ? 'var(--accent-blue)' : 'var(--accent-cyan)' }}>
                                    {v.source || "MOCK"}
                                </strong>
                            </div>
                            <div>
                                <span>Confidence: </span>
                                <strong>{v.confidence ? `${(v.confidence * 100).toFixed(1)}%` : 'N/A'}</strong>
                            </div>
                            <div>
                                <span>Detected by: </span>
                                <strong>D{v.detected_by !== undefined ? v.detected_by : '?'}</strong>
                            </div>
                            <div>
                                <span>Position: </span>
                                <strong>Grid ({v.x}, {v.y})</strong>
                            </div>
                            <div>
                                <span>Loc Error (GT): </span>
                                <strong>
                                    {v.gt_error !== undefined && v.gt_error >= 0 ? `${v.gt_error.toFixed(1)}m` : 'N/A'}
                                </strong>
                            </div>
                            <div style={{ gridColumn: '1 / -1', marginTop: '0.25rem', display: 'flex', justifyContent: 'space-between' }}>
                                <div>
                                    <span style={{ color: 'var(--text-muted)' }}>Last seen: </span>
                                    <strong style={{ color: 'var(--text-main)' }}>{v.last_seen_sec_ago !== undefined ? `${v.last_seen_sec_ago.toFixed(1)} sec ago` : 'N/A'}</strong>
                                </div>
                                <div>
                                    <span style={{ color: 'var(--text-muted)' }}>Observations: </span>
                                    <strong style={{ color: 'var(--text-main)' }}>{v.observations !== undefined ? v.observations : 1}</strong>
                                </div>
                            </div>
                        </div>
                    </div>
                ))}
            </div>
        </div>
    );
}
