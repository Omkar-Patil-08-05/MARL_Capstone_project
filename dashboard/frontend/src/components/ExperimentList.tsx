import React, { useEffect, useState } from 'react';

export interface Experiment {
    id: string;
    type: string;
    name: string;
    timestamp: number;
    environment: string;
    qmix_checkpoint: string | null;
    config_metadata: string;
}

export interface Mission {
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

interface Props {
    onSelectMission: (missionId: string) => void;
}

export function ExperimentList({ onSelectMission }: Props) {
    const [experiments, setExperiments] = useState<Experiment[]>([]);
    const [missions, setMissions] = useState<Mission[]>([]);
    const [selectedExpId, setSelectedExpId] = useState<string | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        fetch('http://localhost:8000/api/db/experiments')
            .then(res => res.json())
            .then(data => {
                setExperiments(data);
                if (data.length > 0) setSelectedExpId(data[0].id);
            })
            .catch(err => setError(err.message))
            .finally(() => setLoading(false));
    }, []);

    useEffect(() => {
        if (!selectedExpId) return;
        fetch(`http://localhost:8000/api/db/missions?experiment_id=${selectedExpId}`)
            .then(res => res.json())
            .then(data => setMissions(data))
            .catch(err => console.error(err));
    }, [selectedExpId]);

    if (loading) return <div className="panel">Loading experiments...</div>;
    if (error) return <div className="panel" style={{ color: 'var(--text-warning)' }}>Error: {error}</div>;
    if (experiments.length === 0) return <div className="panel">No experiments recorded.</div>;

    const selectedExp = experiments.find(e => e.id === selectedExpId);

    return (
        <div style={{ display: 'flex', gap: '16px', flex: 1, minHeight: 0 }}>
            {/* Experiment Sidebar */}
            <div className="panel flex-col" style={{ width: '350px', overflowY: 'auto' }}>
                <div className="panel-title">EXPERIMENTS</div>
                <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', marginTop: '12px' }}>
                    {experiments.map(exp => (
                        <div 
                            key={exp.id} 
                            onClick={() => setSelectedExpId(exp.id)}
                            style={{ 
                                padding: '12px', 
                                border: `1px solid ${selectedExpId === exp.id ? 'var(--accent-cyan)' : 'rgba(255,255,255,0.1)'}`,
                                borderRadius: '4px',
                                cursor: 'pointer',
                                background: selectedExpId === exp.id ? 'rgba(0, 255, 255, 0.05)' : 'transparent'
                            }}
                        >
                            <div style={{ fontWeight: 'bold' }}>{exp.name || exp.id.slice(0,8)}</div>
                            <div className="text-muted" style={{ fontSize: '0.8rem', marginTop: '4px' }}>
                                Type: {exp.type || 'N/A'}
                            </div>
                            <div className="text-muted" style={{ fontSize: '0.8rem' }}>
                                {new Date(exp.timestamp * 1000).toLocaleString()}
                            </div>
                        </div>
                    ))}
                </div>
            </div>

            {/* Mission List */}
            <div className="panel flex-col" style={{ flex: 1, overflowY: 'auto' }}>
                <div className="panel-title">
                    MISSIONS {selectedExp ? `— ${selectedExp.name}` : ''}
                </div>
                {selectedExp && (
                    <div className="text-muted" style={{ fontSize: '0.85rem', marginBottom: '16px', marginTop: '8px' }}>
                        Checkpoint: {selectedExp.qmix_checkpoint || 'N/A'} | Map: {selectedExp.environment}
                    </div>
                )}

                <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.9rem' }}>
                    <thead>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.2)', textAlign: 'left' }}>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Status</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Map</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Drones</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Victims</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Coverage</th>
                            <th style={{ padding: '8px 4px', color: 'var(--text-muted)' }}>Action</th>
                        </tr>
                    </thead>
                    <tbody>
                        {missions.map(m => (
                            <tr key={m.id} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                                <td style={{ padding: '12px 4px', color: m.status === 'COMPLETE' ? 'var(--accent-green)' : 'var(--text-main)' }}>
                                    {m.status}
                                </td>
                                <td style={{ padding: '12px 4px' }}>{m.map_id}</td>
                                <td style={{ padding: '12px 4px' }}>{m.drone_count}</td>
                                <td style={{ padding: '12px 4px' }}>{m.victim_count}</td>
                                <td style={{ padding: '12px 4px', color: 'var(--accent-cyan)' }}>
                                    {m.final_coverage !== null ? `${m.final_coverage}%` : 'N/A'}
                                </td>
                                <td style={{ padding: '12px 4px' }}>
                                    <button 
                                        onClick={() => onSelectMission(m.id)}
                                        style={{ 
                                            background: 'transparent', 
                                            border: '1px solid var(--accent-cyan)', 
                                            color: 'var(--accent-cyan)',
                                            padding: '4px 12px',
                                            borderRadius: '4px',
                                            cursor: 'pointer'
                                        }}
                                    >
                                        View Details
                                    </button>
                                </td>
                            </tr>
                        ))}
                        {missions.length === 0 && (
                            <tr>
                                <td colSpan={6} style={{ padding: '16px', textAlign: 'center', color: 'var(--text-muted)' }}>
                                    No missions recorded for this experiment.
                                </td>
                            </tr>
                        )}
                    </tbody>
                </table>
            </div>
        </div>
    );
}
