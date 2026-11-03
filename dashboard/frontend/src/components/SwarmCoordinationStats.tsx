import React from 'react';
import type { TelemetryPayload } from '../types/telemetry';
import { formatCoverage } from '../utils/formatters';

export function SwarmCoordinationStats({ telemetry }: { telemetry: TelemetryPayload | null }) {
    if (!telemetry || !telemetry.coordination) {
        return (
            <div className="panel" style={{ marginTop: '1rem' }}>
                <h3>SWARM COORDINATION</h3>
                <div style={{ padding: '1rem', textAlign: 'center', color: 'var(--text-muted)' }}>
                    Coordination data unavailable.
                </div>
            </div>
        );
    }

    const coord = telemetry.coordination;
    const activeFrontierCount = Object.keys(coord.active_frontiers || {}).length;
    const currentHolds = Object.keys(coord.safety_holds || {}).length;
    const totalInterventions = telemetry.mission.safety_overrides;

    // Task Allocation Data
    const taskAlloc = telemetry.task_allocation;
    const tasks = telemetry.tasks || [];
    const taskAssignments = coord.task_assignments || taskAlloc?.assignments || taskAlloc?.active_assignments || {};
    const assignmentEntries = Object.entries(taskAssignments);

    const counts = taskAlloc?.counts;
    let activeTasksCount = 0;
    let totalTasksCount = 0;

    if (counts) {
        totalTasksCount = counts.total ?? 0;
        activeTasksCount = (counts.assigned ?? 0) + (counts.approaching ?? 0) + (counts.investigating ?? 0) + (counts.discovered ?? 0);
    } else if (taskAlloc?.active_tasks !== undefined || taskAlloc?.total_tasks !== undefined) {
        activeTasksCount = taskAlloc.active_tasks ?? 0;
        totalTasksCount = taskAlloc.total_tasks ?? 0;
    } else if (tasks.length > 0) {
        totalTasksCount = tasks.length;
        activeTasksCount = tasks.filter(t => t.state !== 'COMPLETED' && t.state !== 'FAILED').length;
    }

    return (
        <div className="panel" style={{ marginTop: '1rem' }}>
            <h3>SWARM COORDINATION</h3>

            <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', fontSize: '0.9rem' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Active swarm size:</span>
                    <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                        <span style={{
                            background: (coord.qmix_drones + coord.coord_drones) === 6 ? '#facc15' : 'var(--accent-cyan)',
                            color: '#000000',
                            fontWeight: 800,
                            fontSize: '0.72rem',
                            padding: '1px 6px',
                            borderRadius: '3px'
                        }}>
                            N = {coord.qmix_drones + coord.coord_drones}
                        </span>
                        <strong style={{ color: 'var(--text-main)' }}>
                            {coord.qmix_drones + coord.coord_drones} {(coord.qmix_drones + coord.coord_drones) === 1 ? 'Drone' : 'Drones'}
                        </strong>
                    </div>
                </div>

                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', padding: '0.5rem', background: 'rgba(255,255,255,0.03)', borderRadius: '4px' }}>
                    <div>
                        <span style={{ color: 'var(--text-muted)', display: 'block', marginBottom: '2px' }}>QMIX agents:</span>
                        {Array.from({length: coord.qmix_drones}).map((_, i) => (
                            <strong key={`q-${i}`} style={{ color: 'var(--accent-blue)', display: 'block' }}>D{i}</strong>
                        ))}
                    </div>
                    <div>
                        <span style={{ color: 'var(--text-muted)', display: 'block', marginBottom: '2px' }}>Coordinated explorers:</span>
                        {Array.from({length: coord.coord_drones}).map((_, i) => (
                            <strong key={`c-${i}`} style={{ color: 'var(--accent-purple)', display: 'block' }}>D{coord.qmix_drones + i}</strong>
                        ))}
                    </div>
                </div>

                {/* SAR Task Allocation Section */}
                <div style={{ borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                        <span style={{ color: 'var(--text-muted)' }}>Autonomous SAR tasks:</span>
                        <strong style={{ color: 'var(--accent-cyan)' }}>
                            {activeTasksCount} active / {totalTasksCount} total
                        </strong>
                    </div>

                    {assignmentEntries.length > 0 ? (
                        <div style={{ paddingLeft: '0.5rem', borderLeft: '2px solid var(--accent-cyan)', marginTop: '4px' }}>
                            {assignmentEntries.map(([droneId, taskId]) => {
                                const matchingTask = tasks.find(t => t.task_id === taskId);
                                const state = matchingTask ? matchingTask.state : 'ASSIGNED';
                                return (
                                    <div key={droneId} style={{ fontSize: '0.8rem', color: 'var(--text-main)', display: 'flex', justifyContent: 'space-between' }}>
                                        <span>{droneId.replace('drone_', 'D')} → {taskId.replace('task_', 'T:')}</span>
                                        <span style={{ fontSize: '0.72rem', color: 'var(--accent-cyan)', fontWeight: 600 }}>{state}</span>
                                    </div>
                                );
                            })}
                        </div>
                    ) : (
                        <div style={{ fontSize: '0.78rem', color: 'var(--text-muted)', fontStyle: 'italic' }}>
                            No active task assignments (exploring)
                        </div>
                    )}
                </div>

                {/* Frontier Reservations */}
                <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Active frontier reservations:</span>
                    <strong style={{ color: 'var(--accent-green)' }}>{activeFrontierCount}</strong>
                </div>

                {activeFrontierCount > 0 && (
                    <div style={{ paddingLeft: '0.5rem', borderLeft: '2px solid rgba(255,255,255,0.1)' }}>
                        {Object.entries(coord.active_frontiers).map(([d_id, [gx, gy]]) => (
                            <div key={d_id} style={{ fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                                {d_id.replace('drone_', 'D')} → frontier ({gx}, {gy})
                            </div>
                        ))}
                    </div>
                )}

                <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Safety interventions:</span>
                    <strong style={{ color: 'var(--text-warning)' }}>{totalInterventions}</strong>
                </div>

                <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Current HOLD drones:</span>
                    <strong style={{ color: currentHolds > 0 ? 'var(--text-warning)' : 'var(--text-main)' }}>{currentHolds}</strong>
                </div>

                <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '0.75rem' }}>
                    <span style={{ color: 'var(--text-muted)' }}>Coverage:</span>
                    <strong style={{ color: 'var(--accent-cyan)' }}>{formatCoverage(telemetry.mission.coverage, 2)}</strong>
                </div>
            </div>
        </div>
    );
}
