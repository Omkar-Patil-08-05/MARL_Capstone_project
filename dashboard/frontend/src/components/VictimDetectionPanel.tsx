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
                        borderLeft: `4px solid ${v.state === 'DETECTED' ? 'var(--accent-green)' : (v.state === 'SEARCHING' ? 'var(--accent-yellow)' : 'var(--text-muted)')}`
                    }}>
                        <div className="flex-row justify-between items-center" style={{ marginBottom: '0.5rem' }}>
                            <strong style={{ fontSize: '1.1rem', color: 'var(--text-main)' }}>{v.id}</strong>
                            <span style={{
                                padding: '2px 6px',
                                borderRadius: '4px',
                                fontSize: '0.75rem',
                                fontWeight: 'bold',
                                background: v.state === 'DETECTED' ? 'rgba(0,255,100,0.1)' : 'rgba(255,255,255,0.1)',
                                color: v.state === 'DETECTED' ? 'var(--accent-green)' : 'var(--text-muted)'
                            }}>
                                STATUS: {v.state || "UNKNOWN"}
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
                            <div style={{ gridColumn: '1 / -1', marginTop: '0.25rem' }}>
                                <span>Last seen: </span>
                                <strong>{v.last_seen_sec_ago !== undefined ? `${v.last_seen_sec_ago.toFixed(1)} sec ago` : 'N/A'}</strong>
                            </div>
                        </div>
                    </div>
                ))}
            </div>
        </div>
    );
}
