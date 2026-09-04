import React, { useEffect, useState } from 'react';

interface TelemetryRow {
    id: number;
    timestamp: number;
    drone_id: string;
    x: number;
    y: number;
    z: number;
    vx: number;
    vy: number;
    vz: number;
    state: string;
    action: string;
}

interface TelemetryResponse {
    items: TelemetryRow[];
    limit: number;
    offset: number;
    total: number;
    has_more: boolean;
}

interface Props {
    missionId: string;
}

export function TelemetryTable({ missionId }: Props) {
    const [data, setData] = useState<TelemetryResponse | null>(null);
    const [offset, setOffset] = useState(0);
    const [limit] = useState(100);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        setLoading(true);
        fetch(`http://localhost:8000/api/db/telemetry?mission_id=${missionId}&limit=${limit}&offset=${offset}`)
            .then(res => res.json())
            .then(resData => setData(resData))
            .catch(err => setError(err.message))
            .finally(() => setLoading(false));
    }, [missionId, offset, limit]);

    if (error) return <div className="panel" style={{ color: 'var(--text-warning)' }}>Error: {error}</div>;

    return (
        <div className="panel flex-col" style={{ flex: 1, minHeight: '400px' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '12px' }}>
                <div className="panel-title">TELEMETRY DATA</div>
                {data && (
                    <div style={{ fontSize: '0.85rem', color: 'var(--text-muted)' }}>
                        Showing {offset + 1}–{Math.min(offset + limit, data.total)} of {data.total}
                    </div>
                )}
            </div>

            <div style={{ overflowX: 'auto', flex: 1, minHeight: 0 }}>
                <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem' }}>
                    <thead style={{ position: 'sticky', top: 0, background: '#111827', zIndex: 1 }}>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.2)', textAlign: 'left' }}>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Time</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Drone</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>X</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Y</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Z</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>State</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Action</th>
                        </tr>
                    </thead>
                    <tbody>
                        {loading && !data && (
                            <tr><td colSpan={7} style={{ padding: '16px', textAlign: 'center' }}>Loading...</td></tr>
                        )}
                        {data?.items.map(row => (
                            <tr key={row.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                                <td style={{ padding: '6px 4px' }}>{row.timestamp.toFixed(2)}</td>
                                <td style={{ padding: '6px 4px' }}>{row.drone_id}</td>
                                <td style={{ padding: '6px 4px' }}>{row.x?.toFixed(2) ?? 'N/A'}</td>
                                <td style={{ padding: '6px 4px' }}>{row.y?.toFixed(2) ?? 'N/A'}</td>
                                <td style={{ padding: '6px 4px' }}>{row.z?.toFixed(2) ?? 'N/A'}</td>
                                <td style={{ padding: '6px 4px', color: 'var(--accent-cyan)' }}>{row.state || 'N/A'}</td>
                                <td style={{ padding: '6px 4px' }}>{row.action || 'N/A'}</td>
                            </tr>
                        ))}
                        {data?.items.length === 0 && (
                            <tr><td colSpan={7} style={{ padding: '16px', textAlign: 'center', color: 'var(--text-muted)' }}>No telemetry recorded.</td></tr>
                        )}
                    </tbody>
                </table>
            </div>

            <div style={{ display: 'flex', justifyContent: 'center', gap: '16px', marginTop: '12px' }}>
                <button 
                    disabled={offset === 0 || loading}
                    onClick={() => setOffset(Math.max(0, offset - limit))}
                    style={{ background: 'var(--panel-bg)', border: '1px solid var(--text-muted)', color: 'white', padding: '4px 12px', borderRadius: '4px', cursor: offset === 0 ? 'not-allowed' : 'pointer' }}
                >
                    Previous
                </button>
                <button 
                    disabled={!data?.has_more || loading}
                    onClick={() => setOffset(offset + limit)}
                    style={{ background: 'var(--panel-bg)', border: '1px solid var(--text-muted)', color: 'white', padding: '4px 12px', borderRadius: '4px', cursor: !data?.has_more ? 'not-allowed' : 'pointer' }}
                >
                    Next
                </button>
            </div>
        </div>
    );
}
