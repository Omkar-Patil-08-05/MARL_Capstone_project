import React from 'react';
import { Target, Search, Users, ShieldAlert } from 'lucide-react';
import type { MissionTelemetry, VictimState } from '../types/telemetry';

interface MissionOverviewProps {
    telemetry: MissionTelemetry | null;
    victims: VictimState[];
}

export function MissionOverview({ telemetry, victims }: MissionOverviewProps) {
    if (!telemetry) return null;

    return (
        <div className="flex-row gap-4">
            {/* Search Coverage */}
            <div className="panel flex-1 flex-col gap-2">
                <div className="flex-row items-center gap-4">
                    <Search className="text-blue" size={24} />
                    <div className="value-display">
                        <span className="value-label">Search Coverage</span>
                        <span className="value-text">{telemetry.coverage.toFixed(1)}%</span>
                    </div>
                </div>
                {telemetry.explored_count != null && telemetry.valid_count != null && (
                    <>
                        <div style={{ fontSize: '0.75rem', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
                            {telemetry.explored_count} / {telemetry.valid_count} cells
                        </div>
                        <div style={{ height: '4px', background: 'rgba(255,255,255,0.05)', borderRadius: '2px', overflow: 'hidden' }}>
                            <div style={{ height: '100%', width: `${Math.min(100, telemetry.coverage)}%`, background: 'var(--accent-blue)', transition: 'width 0.5s ease' }} />
                        </div>
                    </>
                )}
            </div>

            {/* Victims */}
            <div className="panel flex-1 flex-col gap-2">
                <div className="flex-row items-center gap-4">
                    <Users className="text-green" size={24} />
                    <div className="value-display">
                        <span className="value-label">Victims Detected</span>
                        <span className="value-text">{telemetry.victims_detected} / {telemetry.total_victims}</span>
                    </div>
                </div>
                {/* Victim dot tracker */}
                <div style={{ display: 'flex', gap: '6px', marginTop: '2px' }}>
                    {victims.map(v => (
                        <div key={v.id} title={`${v.id} (${v.x}, ${v.y})`} style={{
                            width: 18, height: 18,
                            borderRadius: '50%',
                            display: 'flex', alignItems: 'center', justifyContent: 'center',
                            fontSize: '0.6rem', fontWeight: 700,
                            fontFamily: 'var(--font-mono)',
                            background: v.detected ? 'var(--accent-green)' : 'rgba(255,85,51,0.3)',
                            color: v.detected ? '#000' : '#ff5533',
                            border: v.detected ? '1px solid var(--accent-green)' : '1px solid #ff5533',
                            transition: 'all 0.3s'
                        }}>
                            {v.detected ? '✓' : '?'}
                        </div>
                    ))}
                    {/* Fallback if no victim state yet */}
                    {victims.length === 0 && Array.from({ length: telemetry.total_victims }, (_, i) => (
                        <div key={i} style={{
                            width: 18, height: 18, borderRadius: '50%',
                            display: 'flex', alignItems: 'center', justifyContent: 'center',
                            fontSize: '0.6rem', fontWeight: 700,
                            background: 'rgba(255,255,255,0.06)', color: 'var(--text-muted)', border: '1px solid var(--border-light)'
                        }}>?</div>
                    ))}
                </div>
            </div>

            {/* Decisions */}
            <div className="panel flex-1 flex-row items-center justify-between">
                <div className="flex-row items-center gap-4">
                    <Target className="text-cyan" size={24} />
                    <div className="value-display">
                        <span className="value-label">QMIX Decisions</span>
                        <span className="value-text">{telemetry.decision_count} / {telemetry.max_decisions}</span>
                        {telemetry.global_step_state && (
                            <span style={{ fontSize: '0.65rem', color: telemetry.global_step_state === 'EXECUTING' ? 'var(--accent-cyan)' : 'var(--text-muted)' }}>
                                {telemetry.global_step_state}
                            </span>
                        )}
                    </div>
                </div>
            </div>

            {/* Safety */}
            <div className="panel flex-1 flex-row items-center justify-between">
                <div className="flex-row items-center gap-4">
                    <ShieldAlert className={telemetry.safety_overrides > 0 ? "text-red" : "text-muted"} size={24} />
                    <div className="value-display">
                        <span className="value-label">Safety Overrides</span>
                        <span className="value-text">{telemetry.safety_overrides}</span>
                    </div>
                </div>
            </div>
        </div>
    );
}
