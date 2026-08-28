import React from 'react';
import type { MissionTelemetry } from '../types/telemetry';

export function MissionProgress({ telemetry }: { telemetry: MissionTelemetry | null }) {
    if (!telemetry) return null;

    const decisionPct = Math.min(100, (telemetry.decision_count / telemetry.max_decisions) * 100);
    const coveragePct = Math.min(100, telemetry.coverage);
    const victimPct = Math.min(100, (telemetry.victims_detected / telemetry.total_victims) * 100);

    const coverageStr = telemetry.explored_count != null && telemetry.valid_count != null
        ? `${telemetry.coverage.toFixed(1)}% — ${telemetry.explored_count} / ${telemetry.valid_count} cells`
        : `${telemetry.coverage.toFixed(1)}%`;

    const ProgressBar = ({ label, pct, valueStr, color }: { label: string, pct: number, valueStr: string, color: string }) => (
        <div className="flex-1 flex-col gap-2">
            <div className="flex-row justify-between items-center" style={{ fontSize: '0.75rem', textTransform: 'uppercase', color: 'var(--text-muted)' }}>
                <span>{label}</span>
                <span style={{ fontFamily: 'var(--font-mono)', color: 'var(--text-main)' }}>{valueStr}</span>
            </div>
            <div style={{ height: '6px', background: 'rgba(255,255,255,0.05)', borderRadius: '3px', overflow: 'hidden' }}>
                <div style={{ height: '100%', width: `${pct}%`, background: color, transition: 'width 0.3s ease' }} />
            </div>
        </div>
    );

    return (
        <div className="panel flex-row gap-4">
            <ProgressBar 
                label="QMIX Decisions" 
                pct={decisionPct} 
                valueStr={`${telemetry.decision_count} / ${telemetry.max_decisions}`} 
                color="var(--accent-cyan)" 
            />
            <ProgressBar 
                label="Search Coverage" 
                pct={coveragePct} 
                valueStr={coverageStr} 
                color="var(--accent-blue)" 
            />
            <ProgressBar 
                label="Victims Found" 
                pct={victimPct} 
                valueStr={`${telemetry.victims_detected} / ${telemetry.total_victims}`} 
                color="var(--accent-green)" 
            />
        </div>
    );
}
