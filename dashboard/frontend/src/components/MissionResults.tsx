import React, { useEffect, useState } from 'react';
import type { MissionResult } from '../types/telemetry';

export function MissionResults() {
    const [results, setResults] = useState<MissionResult[]>([]);

    useEffect(() => {
        fetch('http://localhost:8000/api/results')
            .then(res => res.json())
            .then(data => {
                const completed = (data as MissionResult[]).filter(r => r.status === 'COMPLETE');
                setResults(completed);
            })
            .catch(err => console.error("Failed to load results", err));
    }, []);

    const n2Run = [...results].reverse().find(r => r.drone_count === 2);
    const n4Run = [...results].reverse().find(r => r.drone_count === 4);
    const n6Run = [...results].reverse().find(r => r.drone_count === 6);

    if (!n2Run && !n4Run && !n6Run) {
        return (
            <div className="panel flex-col" style={{ flex: 1, minHeight: '250px' }}>
                <div className="panel-title">EVALUATION RESULTS</div>
                <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: '8px' }}>
                    <span className="text-muted" style={{ fontWeight: 'bold' }}>No completed comparison experiment</span>
                    <span className="text-muted" style={{ fontSize: '0.85rem' }}>Run:</span>
                    <div style={{ display: 'flex', gap: '12px', alignItems: 'center' }}>
                        <span style={{ color: 'var(--accent-cyan)', fontWeight: 'bold', border: '1px solid var(--accent-cyan)', padding: '2px 8px', borderRadius: '4px' }}>N = 2</span>
                        <span className="text-muted" style={{ fontSize: '0.85rem' }}>,</span>
                        <span style={{ color: '#a78bfa', fontWeight: 'bold', border: '1px solid #a78bfa', padding: '2px 8px', borderRadius: '4px' }}>N = 4</span>
                        <span className="text-muted" style={{ fontSize: '0.85rem' }}>and</span>
                        <span style={{ color: '#facc15', fontWeight: 'bold', border: '1px solid #facc15', padding: '2px 8px', borderRadius: '4px' }}>N = 6</span>
                    </div>
                    <span className="text-muted" style={{ fontSize: '0.85rem', marginTop: '4px' }}>to generate the comparison.</span>
                </div>
            </div>
        );
    }


    return (
        <div className="panel flex-col" style={{ flex: 1, minHeight: '250px' }}>
            <div className="panel-title">EVALUATION RESULTS</div>

            <div style={{ marginTop: '0.5rem', overflowX: 'auto' }}>
                <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem' }}>
                    <thead>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.1)', textAlign: 'left' }}>
                            <th style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>Metric</th>
                            <th style={{ padding: '6px 4px', color: 'var(--text-main)' }}>N=2</th>
                            <th style={{ padding: '6px 4px', color: 'var(--text-main)' }}>N=4</th>
                            <th style={{ padding: '6px 4px', color: 'var(--text-main)' }}>N=6</th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                            <td style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>Coverage</td>
                            <td style={{ padding: '6px 4px', color: n2Run ? 'var(--accent-cyan)' : 'var(--text-muted)' }}>
                                {n2Run ? `${n2Run.final_coverage}%` : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n4Run ? '#a78bfa' : 'var(--text-muted)' }}>
                                {n4Run ? `${n4Run.final_coverage}%` : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n6Run ? '#facc15' : 'var(--text-muted)' }}>
                                {n6Run ? `${n6Run.final_coverage}%` : 'Planned'}
                            </td>
                        </tr>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                            <td style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>Victims</td>
                            <td style={{ padding: '6px 4px', color: n2Run ? 'var(--accent-green)' : 'var(--text-muted)' }}>
                                {n2Run ? `${n2Run.victims_detected} / 5` : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n4Run ? 'var(--accent-green)' : 'var(--text-muted)' }}>
                                {n4Run ? `${n4Run.victims_detected} / 5` : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n6Run ? 'var(--accent-green)' : 'var(--text-muted)' }}>
                                {n6Run ? `${n6Run.victims_detected} / 5` : 'Planned'}
                            </td>
                        </tr>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                            <td style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>Observations</td>
                            <td style={{ padding: '6px 4px', color: n2Run ? 'var(--text-main)' : 'var(--text-muted)' }}>
                                {n2Run ? n2Run.total_victim_observations : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n4Run ? 'var(--text-main)' : 'var(--text-muted)' }}>
                                {n4Run ? n4Run.total_victim_observations : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n6Run ? 'var(--text-main)' : 'var(--text-muted)' }}>
                                {n6Run ? n6Run.total_victim_observations : 'Planned'}
                            </td>
                        </tr>
                        <tr style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                            <td style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>Safety</td>
                            <td style={{ padding: '6px 4px', color: n2Run ? 'var(--text-warning)' : 'var(--text-muted)' }}>
                                {n2Run ? n2Run.safety_interventions : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n4Run ? 'var(--text-warning)' : 'var(--text-muted)' }}>
                                {n4Run ? n4Run.safety_interventions : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n6Run ? 'var(--text-warning)' : 'var(--text-muted)' }}>
                                {n6Run ? n6Run.safety_interventions : 'Planned'}
                            </td>
                        </tr>
                        <tr>
                            <td style={{ padding: '6px 4px', color: 'var(--text-muted)' }}>Duration</td>
                            <td style={{ padding: '6px 4px', color: n2Run ? 'var(--text-main)' : 'var(--text-muted)' }}>
                                {n2Run ? `${Math.floor(n2Run.mission_duration)}s` : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n4Run ? 'var(--text-main)' : 'var(--text-muted)' }}>
                                {n4Run ? `${Math.floor(n4Run.mission_duration)}s` : 'Planned'}
                            </td>
                            <td style={{ padding: '6px 4px', color: n6Run ? 'var(--text-main)' : 'var(--text-muted)' }}>
                                {n6Run ? `${Math.floor(n6Run.mission_duration)}s` : 'Planned'}
                            </td>
                        </tr>
                    </tbody>
                </table>
            </div>

            <div style={{ marginTop: '0.5rem', flex: 1, position: 'relative' }}>
                <div style={{ display: 'flex', alignItems: 'flex-end', justifyContent: 'space-around', height: '60px', borderBottom: '1px solid rgba(255,255,255,0.2)' }}>
                    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', width: '30px' }}>
                        <span style={{ fontSize: '0.65rem', marginBottom: '2px', color: n2Run ? 'var(--text-main)' : 'transparent' }}>
                            {n2Run ? `${n2Run.final_coverage}%` : ''}
                        </span>
                        <div style={{
                            width: '100%',
                            height: n2Run ? `${n2Run.final_coverage}%` : '0%',
                            background: 'var(--accent-cyan)',
                            borderRadius: '2px 2px 0 0'
                        }} />
                        <span style={{ fontSize: '0.65rem', marginTop: '4px', color: 'var(--text-muted)' }}>N=2</span>
                    </div>

                    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', width: '30px' }}>
                        <span style={{ fontSize: '0.65rem', marginBottom: '2px', color: n4Run ? 'var(--text-main)' : 'transparent' }}>
                            {n4Run ? `${n4Run.final_coverage}%` : ''}
                        </span>
                        <div style={{
                            width: '100%',
                            height: n4Run ? `${n4Run.final_coverage}%` : '0%',
                            background: '#a78bfa',
                            borderRadius: '2px 2px 0 0'
                        }} />
                        <span style={{ fontSize: '0.65rem', marginTop: '4px', color: 'var(--text-muted)' }}>N=4</span>
                    </div>

                    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', width: '30px' }}>
                        <span style={{ fontSize: '0.65rem', marginBottom: '2px', color: n6Run ? 'var(--text-main)' : 'transparent' }}>
                            {n6Run ? `${n6Run.final_coverage}%` : ''}
                        </span>
                        <div style={{
                            width: '100%',
                            height: n6Run ? `${n6Run.final_coverage}%` : '0%',
                            background: '#facc15',
                            borderRadius: '2px 2px 0 0'
                        }} />
                        <span style={{ fontSize: '0.65rem', marginTop: '4px', color: 'var(--text-muted)' }}>N=6</span>
                    </div>
                </div>
            </div>
        </div>
    );
}
