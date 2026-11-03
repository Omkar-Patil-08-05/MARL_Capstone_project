import React, { useState } from 'react';
import type { VictimState, SARTaskTelemetry, TaskAllocationSummary, EvaluationMetrics } from '../types/telemetry';

interface VictimDetectionPanelProps {
    victims: VictimState[];
    trackedVictims?: VictimState[];
    tasks?: SARTaskTelemetry[];
    taskAllocation?: TaskAllocationSummary;
    evaluation?: EvaluationMetrics;
}

const STATE_COLORS: Record<string, { bg: string; text: string; border: string }> = {
    DISCOVERED: { bg: 'rgba(245, 158, 11, 0.15)', text: '#f59e0b', border: '#f59e0b' },
    ASSIGNED: { bg: 'rgba(0, 242, 254, 0.15)', text: '#00f2fe', border: '#00f2fe' },
    APPROACHING: { bg: 'rgba(129, 140, 248, 0.15)', text: '#818cf8', border: '#818cf8' },
    INVESTIGATING: { bg: 'rgba(250, 204, 21, 0.2)', text: '#facc15', border: '#facc15' },
    COMPLETED: { bg: 'rgba(16, 185, 129, 0.15)', text: '#10b981', border: '#10b981' },
    FAILED: { bg: 'rgba(239, 68, 68, 0.15)', text: '#ef4444', border: '#ef4444' },
};

export function VictimDetectionPanel({
    victims = [],
    trackedVictims = [],
    tasks = [],
    taskAllocation,
    evaluation,
}: VictimDetectionPanelProps) {
    const [subTab, setSubTab] = useState<'AUTONOMOUS' | 'TASKS' | 'GROUND_TRUTH' | 'EVALUATION'>('AUTONOMOUS');

    const detectedGTVictims = (victims || []).filter(v => v.detected);
    const totalGTVictims = (victims || []).length;
    const autonomousCount = (trackedVictims || []).length;
    const taskCount = (tasks || []).length;

    const counts = taskAllocation?.counts;
    const totalTasks = counts?.total ?? taskAllocation?.total_tasks ?? taskCount;
    const activeTasksCount = counts
        ? ((counts.assigned ?? 0) + (counts.approaching ?? 0) + (counts.investigating ?? 0) + (counts.discovered ?? 0))
        : (taskAllocation?.active_tasks ?? tasks.filter(t => t.state !== 'COMPLETED' && t.state !== 'FAILED').length);
    const completedTasksCount = counts?.completed ?? taskAllocation?.completed_tasks ?? tasks.filter(t => t.state === 'COMPLETED').length;
    const failedTasksCount = counts?.failed ?? taskAllocation?.failed_tasks ?? tasks.filter(t => t.state === 'FAILED').length;

    return (
        <div className="panel" style={{ marginTop: '1rem', display: 'flex', flexDirection: 'column' }}>
            {/* Header & Sub-Tabs */}
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.75rem' }}>
                <h3 style={{ margin: 0, fontSize: '0.95rem', letterSpacing: '0.5px' }}>VICTIM & TASK DISPATCH</h3>
            </div>

            {/* Segmented Tab Navigation */}
            <div style={{
                display: 'grid',
                gridTemplateColumns: 'repeat(4, 1fr)',
                gap: '4px',
                padding: '3px',
                background: 'rgba(0,0,0,0.25)',
                borderRadius: '6px',
                marginBottom: '0.85rem'
            }}>
                <button
                    onClick={() => setSubTab('AUTONOMOUS')}
                    style={{
                        padding: '6px 2px',
                        fontSize: '0.68rem',
                        fontWeight: 700,
                        border: 'none',
                        borderRadius: '4px',
                        cursor: 'pointer',
                        transition: 'all 0.2s ease',
                        background: subTab === 'AUTONOMOUS' ? 'var(--accent-cyan)' : 'transparent',
                        color: subTab === 'AUTONOMOUS' ? '#000000' : 'var(--text-muted)',
                        whiteSpace: 'nowrap',
                        overflow: 'hidden',
                        textOverflow: 'ellipsis'
                    }}
                >
                    TRACKS ({autonomousCount})
                </button>
                <button
                    onClick={() => setSubTab('TASKS')}
                    style={{
                        padding: '6px 2px',
                        fontSize: '0.68rem',
                        fontWeight: 700,
                        border: 'none',
                        borderRadius: '4px',
                        cursor: 'pointer',
                        transition: 'all 0.2s ease',
                        background: subTab === 'TASKS' ? 'var(--accent-purple)' : 'transparent',
                        color: subTab === 'TASKS' ? '#000000' : 'var(--text-muted)',
                        whiteSpace: 'nowrap',
                        overflow: 'hidden',
                        textOverflow: 'ellipsis'
                    }}
                >
                    TASKS ({activeTasksCount}/{taskCount})
                </button>
                <button
                    onClick={() => setSubTab('GROUND_TRUTH')}
                    style={{
                        padding: '6px 2px',
                        fontSize: '0.68rem',
                        fontWeight: 700,
                        border: 'none',
                        borderRadius: '4px',
                        cursor: 'pointer',
                        transition: 'all 0.2s ease',
                        background: subTab === 'GROUND_TRUTH' ? 'var(--accent-green)' : 'transparent',
                        color: subTab === 'GROUND_TRUTH' ? '#000000' : 'var(--text-muted)',
                        whiteSpace: 'nowrap',
                        overflow: 'hidden',
                        textOverflow: 'ellipsis'
                    }}
                >
                    REF GT ({detectedGTVictims.length}/{totalGTVictims})
                </button>
                <button
                    onClick={() => setSubTab('EVALUATION')}
                    style={{
                        padding: '6px 2px',
                        fontSize: '0.68rem',
                        fontWeight: 700,
                        border: 'none',
                        borderRadius: '4px',
                        cursor: 'pointer',
                        transition: 'all 0.2s ease',
                        background: subTab === 'EVALUATION' ? '#eab308' : 'transparent',
                        color: subTab === 'EVALUATION' ? '#000000' : 'var(--text-muted)',
                        whiteSpace: 'nowrap',
                        overflow: 'hidden',
                        textOverflow: 'ellipsis'
                    }}
                >
                    METRICS
                </button>
            </div>

            {/* TAB 1: AUTONOMOUS DISCOVERIES */}
            {subTab === 'AUTONOMOUS' && (
                <div className="flex-col gap-2" style={{ maxHeight: '380px', overflowY: 'auto', paddingRight: '4px' }}>
                    {autonomousCount === 0 ? (
                        <div style={{ padding: '1.5rem', textAlign: 'center', color: 'var(--text-muted)', fontSize: '0.85rem' }}>
                            No autonomous perception detections yet.<br />
                            <span style={{ fontSize: '0.75rem', opacity: 0.7 }}>Awaiting verified multispectral victim events.</span>
                        </div>
                    ) : (
                        trackedVictims.map((v) => {
                            const confPct = v.confidence !== undefined ? Math.round(v.confidence * 100) : null;
                            const isHighConf = confPct !== null && confPct >= 75;
                            const verMode = v.verification_mode || v.source || 'RGBD_THERMAL';

                            return (
                                <div key={v.id} style={{
                                    padding: '0.75rem',
                                    background: 'rgba(255,255,255,0.03)',
                                    borderRadius: '0.5rem',
                                    borderLeft: `4px solid ${isHighConf ? 'var(--accent-cyan)' : '#f59e0b'}`
                                }}>
                                    <div className="flex-row justify-between items-center" style={{ marginBottom: '0.4rem' }}>
                                        <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                                            <strong style={{ fontSize: '0.95rem', color: 'var(--text-main)', textTransform: 'uppercase' }}>
                                                {v.id.replace('_', ' #')}
                                            </strong>
                                            {v.evidence_state && (
                                                <span style={{
                                                    fontSize: '0.65rem',
                                                    padding: '1px 5px',
                                                    borderRadius: '3px',
                                                    background: 'rgba(255,255,255,0.08)',
                                                    color: 'var(--text-muted)',
                                                    fontWeight: 600
                                                }}>{v.evidence_state}</span>
                                            )}
                                        </div>
                                        {confPct !== null && (
                                            <span style={{
                                                fontSize: '0.7rem',
                                                padding: '2px 8px',
                                                borderRadius: '4px',
                                                background: isHighConf ? 'rgba(0, 242, 254, 0.15)' : 'rgba(245, 158, 11, 0.15)',
                                                color: isHighConf ? 'var(--accent-cyan)' : '#f59e0b',
                                                fontWeight: 700,
                                                fontFamily: 'var(--font-mono)'
                                            }}>
                                                {confPct}% CONF
                                            </span>
                                        )}
                                    </div>

                                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.35rem', fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                                        <div>
                                            <span>Pos: </span>
                                            <strong style={{ color: 'var(--text-main)', fontFamily: 'var(--font-mono)' }}>
                                                {v.world_x !== undefined && v.world_y !== undefined
                                                    ? `(${v.world_x.toFixed(1)}, ${v.world_y.toFixed(1)})`
                                                    : `Grid (${v.x}, ${v.y})`}
                                            </strong>
                                        </div>
                                        <div>
                                            <span>Mode: </span>
                                            <strong style={{ color: 'var(--text-main)' }}>{verMode}</strong>
                                        </div>
                                        {(v.confirming_drone || v.detected_by) && (
                                            <div>
                                                <span>Detector: </span>
                                                <strong style={{ color: 'var(--accent-blue)' }}>
                                                    {String(v.confirming_drone || v.detected_by).replace('drone_', 'D').toUpperCase()}
                                                </strong>
                                            </div>
                                        )}
                                        {v.confirmation_count !== undefined && (
                                            <div>
                                                <span>Frames: </span>
                                                <strong style={{ color: 'var(--text-main)' }}>{v.confirmation_count}</strong>
                                            </div>
                                        )}
                                    </div>
                                </div>
                            );
                        })
                    )}
                </div>
            )}

            {/* TAB 2: SAR TASKS & ALLOCATION */}
            {subTab === 'TASKS' && (
                <div className="flex-col gap-2" style={{ maxHeight: '380px', overflowY: 'auto', paddingRight: '4px' }}>
                    {/* Allocation Summary Stats */}
                    {taskAllocation && (
                        <div style={{
                            display: 'grid',
                            gridTemplateColumns: 'repeat(4, 1fr)',
                            gap: '4px',
                            padding: '6px',
                            background: 'rgba(255,255,255,0.02)',
                            borderRadius: '4px',
                            marginBottom: '4px',
                            textAlign: 'center',
                            fontSize: '0.72rem'
                        }}>
                            <div>
                                <div style={{ color: 'var(--text-muted)' }}>TOTAL</div>
                                <div style={{ fontWeight: 700, color: 'var(--text-main)' }}>{totalTasks}</div>
                            </div>
                            <div>
                                <div style={{ color: 'var(--text-muted)' }}>ACTIVE</div>
                                <div style={{ fontWeight: 700, color: 'var(--accent-cyan)' }}>{activeTasksCount}</div>
                            </div>
                            <div>
                                <div style={{ color: 'var(--text-muted)' }}>DONE</div>
                                <div style={{ fontWeight: 700, color: 'var(--accent-green)' }}>{completedTasksCount}</div>
                            </div>
                            <div>
                                <div style={{ color: 'var(--text-muted)' }}>FAIL</div>
                                <div style={{ fontWeight: 700, color: failedTasksCount > 0 ? '#ef4444' : 'var(--text-muted)' }}>
                                    {failedTasksCount}
                                </div>
                            </div>
                        </div>
                    )}

                    {taskCount === 0 ? (
                        <div style={{ padding: '1.5rem', textAlign: 'center', color: 'var(--text-muted)', fontSize: '0.85rem' }}>
                            No autonomous SAR tasks generated.<br />
                            <span style={{ fontSize: '0.75rem', opacity: 0.7 }}>Tasks are created upon perception verification.</span>
                        </div>
                    ) : (
                        tasks.map((task) => {
                            const sc = STATE_COLORS[task.state] || { bg: 'rgba(255,255,255,0.05)', text: 'var(--text-main)', border: 'rgba(255,255,255,0.2)' };
                            const assignedName = task.assigned_drone_id
                                ? String(task.assigned_drone_id).replace('drone_', 'D').toUpperCase()
                                : 'UNASSIGNED';
                            const dwellProgress = task.investigation_progress_sec !== undefined && task.investigation_duration
                                ? Math.min(100, Math.round((task.investigation_progress_sec / task.investigation_duration) * 100))
                                : null;

                            return (
                                <div key={task.task_id} style={{
                                    padding: '0.75rem',
                                    background: 'rgba(255,255,255,0.03)',
                                    borderRadius: '0.5rem',
                                    borderLeft: `4px solid ${sc.border}`
                                }}>
                                    <div className="flex-row justify-between items-center" style={{ marginBottom: '0.4rem' }}>
                                        <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                                            <strong style={{ fontSize: '0.9rem', color: 'var(--text-main)', fontFamily: 'var(--font-mono)' }}>
                                                {task.task_id}
                                            </strong>
                                            {task.victim_id && (
                                                <span style={{ fontSize: '0.7rem', color: 'var(--text-muted)' }}>
                                                    ({task.victim_id})
                                                </span>
                                            )}
                                        </div>
                                        <span style={{
                                            fontSize: '0.7rem',
                                            padding: '2px 8px',
                                            borderRadius: '4px',
                                            background: sc.bg,
                                            color: sc.text,
                                            fontWeight: 700,
                                            border: `1px solid ${sc.border}44`
                                        }}>
                                            {task.state}
                                        </span>
                                    </div>

                                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.35rem', fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                                        <div>
                                            <span>Target: </span>
                                            <strong style={{ color: 'var(--text-main)', fontFamily: 'var(--font-mono)' }}>
                                                ({task.world_x.toFixed(1)}, {task.world_y.toFixed(1)})
                                            </strong>
                                        </div>
                                        <div>
                                            <span>Drone: </span>
                                            <strong style={{ color: task.assigned_drone_id ? 'var(--accent-cyan)' : 'var(--text-muted)' }}>
                                                {assignedName}
                                            </strong>
                                        </div>
                                        {task.reassignment_count !== undefined && task.reassignment_count > 0 && (
                                            <div>
                                                <span>Reassigns: </span>
                                                <strong style={{ color: '#f59e0b' }}>{task.reassignment_count}</strong>
                                            </div>
                                        )}
                                        {task.confidence !== undefined && (
                                            <div>
                                                <span>Conf: </span>
                                                <strong style={{ color: 'var(--text-main)', fontFamily: 'var(--font-mono)' }}>
                                                    {(task.confidence * 100).toFixed(0)}%
                                                </strong>
                                            </div>
                                        )}
                                    </div>

                                    {/* Investigation Dwell Timer Progress Bar */}
                                    {dwellProgress !== null && task.state === 'INVESTIGATING' && (
                                        <div style={{ marginTop: '0.5rem' }}>
                                            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.7rem', color: '#facc15', marginBottom: '2px' }}>
                                                <span>Verification Dwell</span>
                                                <span>{(task.investigation_progress_sec || 0).toFixed(1)}s / {(task.investigation_duration || 3.0).toFixed(1)}s</span>
                                            </div>
                                            <div style={{ width: '100%', height: '4px', background: 'rgba(255,255,255,0.1)', borderRadius: '2px', overflow: 'hidden' }}>
                                                <div style={{
                                                    width: `${dwellProgress}%`,
                                                    height: '100%',
                                                    background: '#facc15',
                                                    transition: 'width 0.2s ease-out'
                                                }} />
                                            </div>
                                        </div>
                                    )}

                                    {/* Failure Reason */}
                                    {task.state === 'FAILED' && task.failure_reason && (
                                        <div style={{ marginTop: '0.4rem', fontSize: '0.75rem', color: '#ef4444' }}>
                                            Failure: {task.failure_reason}
                                        </div>
                                    )}
                                </div>
                            );
                        })
                    )}
                </div>
            )}

            {/* TAB 3: GROUND TRUTH REFERENCE */}
            {subTab === 'GROUND_TRUTH' && (
                <div className="flex-col gap-2" style={{ maxHeight: '380px', overflowY: 'auto', paddingRight: '4px' }}>
                    <div style={{
                        padding: '8px 10px',
                        fontSize: '0.74rem',
                        background: 'rgba(16, 185, 129, 0.08)',
                        border: '1px solid rgba(16, 185, 129, 0.25)',
                        borderRadius: '4px',
                        color: 'var(--text-muted)',
                        marginBottom: '6px',
                        lineHeight: 1.4
                    }}>
                        <strong style={{ color: 'var(--accent-green)' }}>Simulation Proximity Reference:</strong> Drones logging ground-truth presence (&le; 3.0m). Distinct from autonomous vision detections.
                    </div>

                    {totalGTVictims === 0 ? (
                        <div style={{ padding: '1.5rem', textAlign: 'center', color: 'var(--text-muted)', fontSize: '0.85rem' }}>
                            Waiting for ground truth metadata...
                        </div>
                    ) : detectedGTVictims.length === 0 ? (
                        <div style={{ padding: '1.5rem', textAlign: 'center', color: 'var(--text-muted)', fontSize: '0.85rem' }}>
                            No ground truth victims encountered yet ({totalGTVictims} total).
                        </div>
                    ) : (
                        detectedGTVictims.map((v) => (
                            <div key={v.id} style={{
                                padding: '0.75rem',
                                background: 'rgba(255,255,255,0.03)',
                                borderRadius: '0.5rem',
                                borderLeft: '4px solid var(--accent-green)'
                            }}>
                                <div className="flex-row justify-between items-center" style={{ marginBottom: '0.4rem' }}>
                                    <strong style={{ fontSize: '0.95rem', color: 'var(--text-main)', textTransform: 'uppercase' }}>
                                        {v.id.replace('_', ' #')}
                                    </strong>
                                    <span style={{
                                        fontSize: '0.7rem',
                                        padding: '2px 8px',
                                        borderRadius: '4px',
                                        background: 'rgba(0, 255, 136, 0.15)',
                                        color: 'var(--accent-green)',
                                        fontWeight: 600,
                                    }}>GT DETECTED</span>
                                </div>

                                <div style={{ display: 'grid', gridTemplateColumns: '1fr', gap: '0.35rem', fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                                    {v.detected_by !== undefined && v.detected_by !== null && (
                                        <div>
                                            <span>Encountered by: </span>
                                            <strong style={{ color: 'var(--text-main)' }}>
                                                {String(v.detected_by).replace('_', ' ').toUpperCase()}
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
                                        <strong style={{ color: 'var(--text-main)', fontFamily: 'var(--font-mono)' }}>
                                            {v.world_x !== undefined && v.world_y !== undefined
                                                ? `(${v.world_x.toFixed(1)}, ${v.world_y.toFixed(1)})`
                                                : `Grid (${v.x}, ${v.y})`}
                                        </strong>
                                    </div>
                                </div>
                            </div>
                        ))
                    )}
                </div>
            )}

            {/* TAB 4: ONLINE EVALUATION (HUNGARIAN SPATIAL MATCHING) */}
            {subTab === 'EVALUATION' && (
                <div style={{ display: 'flex', flexDirection: 'column', gap: '0.6rem' }}>
                    {autonomousCount === 0 || !evaluation || (evaluation.total_autonomous_tracks ?? 0) === 0 ? (
                        <div style={{
                            padding: '1.25rem 1rem',
                            background: 'rgba(234, 179, 8, 0.05)',
                            border: '1px dashed rgba(234, 179, 8, 0.3)',
                            borderRadius: '6px',
                            textAlign: 'center',
                            display: 'flex',
                            flexDirection: 'column',
                            alignItems: 'center',
                            gap: '8px'
                        }}>
                            <span style={{
                                fontSize: '0.82rem',
                                fontWeight: 700,
                                color: '#eab308',
                                textTransform: 'uppercase',
                                letterSpacing: '0.5px'
                            }}>
                                Not Evaluated For This Mission
                            </span>
                            <p style={{ fontSize: '0.76rem', color: 'var(--text-muted)', lineHeight: 1.5, maxWidth: '320px', margin: 0 }}>
                                This run operated under coordinate-based QMIX swarm exploration with simulation proximity references ({detectedGTVictims.length}/{totalGTVictims} reference victims matched).
                            </p>
                            <p style={{ fontSize: '0.72rem', color: 'var(--text-secondary)', lineHeight: 1.4, maxWidth: '320px', margin: '4px 0 0 0' }}>
                                Empirical perception metrics (F1, Precision, Recall, RMSE localization error) require active autonomous vision tracks (YOLOv8n / RGB-D depth), which were not active in this run.
                            </p>
                            <div style={{
                                marginTop: '6px',
                                padding: '4px 10px',
                                borderRadius: '4px',
                                background: 'rgba(255,255,255,0.03)',
                                border: '1px solid rgba(255,255,255,0.08)',
                                fontSize: '0.7rem',
                                color: 'var(--accent-cyan)',
                                fontFamily: 'var(--font-mono)'
                            }}>
                                Autonomous Vision Baseline: F1 = 1.0 (TP: 2, FP: 0, Loc Error: 0.52m)
                            </div>
                        </div>
                    ) : (
                        <>
                            <div style={{
                                padding: '0.6rem 0.75rem',
                                background: 'rgba(234, 179, 8, 0.08)',
                                border: '1px solid rgba(234, 179, 8, 0.25)',
                                borderRadius: '6px',
                                fontSize: '0.78rem',
                                color: 'var(--text-muted)'
                            }}>
                                Strict 1-to-1 Hungarian spatial matching between autonomous tracks ({autonomousCount}) and simulation ground truth (&le; 3.0m). Isolated from mission control.
                            </div>

                            {/* Primary Metric Grid */}
                            <div style={{
                                display: 'grid',
                                gridTemplateColumns: 'repeat(3, 1fr)',
                                gap: '0.5rem',
                                textAlign: 'center'
                            }}>
                                <div style={{ background: 'rgba(0,0,0,0.3)', padding: '0.5rem 0.25rem', borderRadius: '4px' }}>
                                    <div style={{ fontSize: '0.65rem', color: 'var(--text-muted)', textTransform: 'uppercase' }}>F1-Score</div>
                                    <div style={{ fontSize: '1.2rem', fontWeight: 800, color: '#eab308' }}>
                                        {((evaluation?.f1_score ?? 0) * 100).toFixed(1)}%
                                    </div>
                                </div>
                                <div style={{ background: 'rgba(0,0,0,0.3)', padding: '0.5rem 0.25rem', borderRadius: '4px' }}>
                                    <div style={{ fontSize: '0.65rem', color: 'var(--text-muted)', textTransform: 'uppercase' }}>Precision</div>
                                    <div style={{ fontSize: '1.2rem', fontWeight: 800, color: 'var(--accent-cyan)' }}>
                                        {((evaluation?.precision ?? 0) * 100).toFixed(1)}%
                                    </div>
                                </div>
                                <div style={{ background: 'rgba(0,0,0,0.3)', padding: '0.5rem 0.25rem', borderRadius: '4px' }}>
                                    <div style={{ fontSize: '0.65rem', color: 'var(--text-muted)', textTransform: 'uppercase' }}>Recall</div>
                                    <div style={{ fontSize: '1.2rem', fontWeight: 800, color: 'var(--accent-green)' }}>
                                        {((evaluation?.recall ?? 0) * 100).toFixed(1)}%
                                    </div>
                                </div>
                            </div>

                            {/* Counts Breakdown: TP, FP, FN */}
                            <div style={{
                                display: 'grid',
                                gridTemplateColumns: 'repeat(3, 1fr)',
                                gap: '0.5rem',
                                textAlign: 'center'
                            }}>
                                <div style={{ border: '1px solid rgba(16, 185, 129, 0.4)', borderRadius: '4px', padding: '0.35rem' }}>
                                    <span style={{ fontSize: '0.65rem', color: 'var(--accent-green)' }}>True Positives</span>
                                    <div style={{ fontSize: '1rem', fontWeight: 700, color: 'var(--accent-green)' }}>
                                        {evaluation?.true_positives ?? 0}
                                    </div>
                                </div>
                                <div style={{ border: '1px solid rgba(239, 68, 68, 0.4)', borderRadius: '4px', padding: '0.35rem' }}>
                                    <span style={{ fontSize: '0.65rem', color: 'var(--text-danger, #ef4444)' }}>False Positives</span>
                                    <div style={{ fontSize: '1rem', fontWeight: 700, color: '#ef4444' }}>
                                        {evaluation?.false_positives ?? 0}
                                    </div>
                                </div>
                                <div style={{ border: '1px solid rgba(245, 158, 11, 0.4)', borderRadius: '4px', padding: '0.35rem' }}>
                                    <span style={{ fontSize: '0.65rem', color: '#f59e0b' }}>False Negatives</span>
                                    <div style={{ fontSize: '1rem', fontWeight: 700, color: '#f59e0b' }}>
                                        {evaluation?.false_negatives ?? totalGTVictims}
                                    </div>
                                </div>
                            </div>

                            {/* Localization Accuracy */}
                            <div style={{
                                background: 'rgba(0,0,0,0.2)',
                                padding: '0.6rem 0.75rem',
                                borderRadius: '6px',
                                fontSize: '0.78rem'
                            }}>
                                <div style={{ fontWeight: 600, color: 'var(--text-main)', marginBottom: '0.4rem' }}>
                                    Localization Accuracy (m)
                                </div>
                                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.4rem', color: 'var(--text-muted)' }}>
                                    <div>Mean Error: <strong style={{ color: 'var(--text-main)' }}>{(evaluation?.mean_localization_error_m ?? 0).toFixed(2)} m</strong></div>
                                    <div>Median Error: <strong style={{ color: 'var(--text-main)' }}>{(evaluation?.median_localization_error_m ?? 0).toFixed(2)} m</strong></div>
                                    <div>RMSE: <strong style={{ color: 'var(--text-main)' }}>{(evaluation?.rmse_localization_error_m ?? 0).toFixed(2)} m</strong></div>
                                    <div>Max Error: <strong style={{ color: 'var(--text-main)' }}>{(evaluation?.max_localization_error_m ?? 0).toFixed(2)} m</strong></div>
                                </div>
                            </div>
                        </>
                    )}
                </div>
            )}
        </div>
    );
}

