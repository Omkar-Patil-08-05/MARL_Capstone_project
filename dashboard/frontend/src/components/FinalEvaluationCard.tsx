import React, { useEffect, useState } from 'react';

interface AuthoritativeEvaluation {
    episodes: number;
    coverage: number;
    coverage_sd: number;
    victims_found_mean: number;
    total_victims: number;
    mean_duration: number;
    duration_sd: number;
    policy_steps: number;
    invalid_actions: number;
    timeouts: number;
    collision_data: string | null;
    error?: string;
}

export function FinalEvaluationCard() {
    const [evalData, setEvalData] = useState<AuthoritativeEvaluation | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        fetch('http://localhost:8000/api/db/evaluation/authoritative')
            .then(res => {
                if (!res.ok) throw new Error("Failed to fetch authoritative evaluation");
                return res.json();
            })
            .then(data => {
                if (data.error) throw new Error(data.error);
                setEvalData(data);
            })
            .catch(err => setError(err.message))
            .finally(() => setLoading(false));
    }, []);

    if (loading) return <div className="panel">Loading authoritative evaluation...</div>;
    if (error) return <div className="panel" style={{ color: 'var(--text-warning)' }}>Error: {error}</div>;
    if (!evalData) return null;

    return (
        <div className="panel flex-col" style={{ flex: 1 }}>
            <div className="panel-title" style={{ color: 'var(--accent-cyan)' }}>
                AUTHORITATIVE QMIX EVALUATION (EPISODES 11–20)
            </div>
            <div style={{ fontSize: '0.85rem', color: 'var(--text-muted)', marginBottom: '1rem' }}>
                {evalData.episodes} raw episodes recorded from Gazebo SITL dataset. Visual perception artifacts bypass enabled.
            </div>

            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '1rem' }}>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Mean Coverage</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold' }}>{(evalData.coverage <= 1.0 ? evalData.coverage * 100 : evalData.coverage).toFixed(2)}% (±{(evalData.coverage_sd <= 1.0 ? evalData.coverage_sd * 100 : evalData.coverage_sd).toFixed(2)})</span>
                </div>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Victim Success</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold', color: 'var(--accent-green)' }}>
                        {evalData.victims_found_mean} / {evalData.total_victims}
                    </span>
                </div>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Mean Duration</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold' }}>
                        {evalData.mean_duration}s (±{evalData.duration_sd})
                    </span>
                </div>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Policy Steps</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold' }}>{evalData.policy_steps}</span>
                </div>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Invalid Actions</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold' }}>{evalData.invalid_actions}</span>
                </div>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Timeouts</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold' }}>{evalData.timeouts}</span>
                </div>
                <div style={{ display: 'flex', flexDirection: 'column' }}>
                    <span className="text-muted" style={{ fontSize: '0.8rem' }}>Collision Data</span>
                    <span style={{ fontSize: '1.2rem', fontWeight: 'bold', color: 'var(--text-muted)' }}>
                        {evalData.collision_data === null ? 'Unavailable' : evalData.collision_data}
                    </span>
                </div>
            </div>
        </div>
    );
}
