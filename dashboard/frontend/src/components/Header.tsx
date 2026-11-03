import React, { useState, useEffect } from 'react';
import { Activity, Radio, WifiOff, Square, RefreshCcw, Map, Clock } from 'lucide-react';
import type { MissionTelemetry, BackendMissionStatus } from '../types/telemetry';

const Timer = ({ simTime }: { simTime?: number }) => {
    const [seconds, setSeconds] = useState(0);
    useEffect(() => {
        const interval = setInterval(() => setSeconds(s => s + 1), 1000);
        return () => clearInterval(interval);
    }, []);

    let simFormatted: string | null = null;
    if (simTime !== undefined && simTime > 0) {
        const sm = Math.floor(simTime / 60);
        const ss = Math.floor(simTime % 60);
        simFormatted = `${sm.toString().padStart(2, '0')}:${ss.toString().padStart(2, '0')}`;
    }

    const m = Math.floor(seconds / 60);
    const s = seconds % 60;
    const wallFormatted = `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;

    return (
        <span className="value-text" style={{ fontSize: '0.95rem', fontFamily: 'var(--font-mono)', display: 'flex', alignItems: 'center', gap: '6px' }}>
            <Clock size={16} className="text-cyan" />
            {simFormatted ? (
                <span>
                    {simFormatted} <span style={{ fontSize: '0.7rem', color: 'var(--text-muted)' }}>(Wall {wallFormatted})</span>
                </span>
            ) : (
                wallFormatted
            )}
        </span>
    );
};

interface HeaderProps {
    isConnected: boolean;
    telemetry: MissionTelemetry | null;
    droneCount?: number;
    backendStatus: BackendMissionStatus;
    onStop: () => void;
    onComplete: () => void;
    onReset: () => void;
    currentView: 'MISSION' | 'DATABASE';
    onToggleView: () => void;
}

export function Header({ isConnected, telemetry, droneCount, backendStatus, onStop, onComplete, onReset, currentView, onToggleView }: HeaderProps) {
    const isLive = isConnected && telemetry !== null;

    return (
        <header className="panel flex-row justify-between items-center">
            <div className="flex-row items-center gap-4">
                <Activity className="text-cyan" size={28} />
                <div className="flex-col">
                    <h1 className="text-xl" style={{ margin: 0 }}>MARL SWARM COMMAND</h1>
                    <span className="text-muted" style={{ fontSize: '0.8rem', letterSpacing: '1px' }}>
                        SEARCH AND RESCUE TELEMETRY
                    </span>
                </div>
            </div>

            <div className="flex-row gap-4">
                {droneCount !== undefined && droneCount > 0 && (
                    <div className="value-display" style={{ alignItems: 'flex-end', marginRight: '24px' }}>
                        <span className="value-label">Active Swarm</span>
                        <span className="value-text" style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                            <span style={{
                                background: droneCount === 6 ? '#facc15' : 'var(--accent-cyan)',
                                color: '#000000',
                                fontWeight: 800,
                                fontSize: '0.72rem',
                                padding: '2px 6px',
                                borderRadius: '4px'
                            }}>
                                N = {droneCount}
                            </span>
                            <span style={{ color: droneCount === 6 ? '#facc15' : 'var(--accent-cyan)', fontSize: '0.85rem', fontWeight: 600 }}>
                                {droneCount === 6 ? '6-Drone Swarm' : droneCount === 2 ? '2-Drone Autonomous' : `${droneCount} Drones`}
                            </span>
                        </span>
                    </div>
                )}

                <div className="value-display" style={{ alignItems: 'flex-end', marginRight: '24px' }}>
                    <span className="value-label">Backend Status</span>
                    <span className="value-text text-cyan">{backendStatus.state}</span>
                </div>

                <div className="value-display" style={{ alignItems: 'flex-end', marginRight: '24px' }}>
                    <span className="value-label">Mission Status</span>
                    <span className="value-text text-cyan">{telemetry ? telemetry.status : backendStatus.state}</span>
                </div>

                {telemetry && telemetry.status === 'RUNNING' && (
                    <div className="value-display" style={{ alignItems: 'flex-end', marginRight: '24px' }}>
                        <span className="value-label">Mission Timer (Sim / Wall)</span>
                        <Timer simTime={telemetry.sim_time} />
                    </div>
                )}

                {backendStatus.state !== 'IDLE' && (
                    <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
                        <button
                            className="btn-danger"
                            onClick={() => { if(window.confirm('Are you sure you want to STOP the mission?')) onStop() }}
                            disabled={backendStatus.state === 'STOPPING' || backendStatus.state === 'COMPLETE'}
                            style={{ display: 'flex', alignItems: 'center', gap: '4px', padding: '6px 12px', background: '#dc2626', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                        >
                            <Square size={14} /> STOP
                        </button>
                        <button
                            className="btn-success"
                            onClick={() => { if(window.confirm('Are you sure you want to COMPLETE the mission?')) onComplete() }}
                            disabled={backendStatus.state === 'STOPPING' || backendStatus.state === 'COMPLETE'}
                            style={{ display: 'flex', alignItems: 'center', gap: '4px', padding: '6px 12px', background: '#059669', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer', fontWeight: 'bold' }}
                        >
                            COMPLETE MISSION
                        </button>
                        <button
                            className="btn-warning"
                            onClick={() => { if(window.confirm('Reset mission?')) onReset() }}
                            style={{ display: 'flex', alignItems: 'center', gap: '4px', padding: '6px 12px', background: '#d97706', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                        >
                            <RefreshCcw size={14} /> RESET
                        </button>
                        <button
                            onClick={async () => {
                                try {
                                    const res = await fetch('http://localhost:8000/api/mission/view_simulation', { method: 'POST' });
                                    if (!res.ok) alert("Failed to launch Gazebo GUI. Alternatively, run: gz sim -g");
                                } catch {
                                    alert("Failed to connect to backend. Run manually: gz sim -g");
                                }
                            }}
                            style={{ display: 'flex', alignItems: 'center', gap: '6px', padding: '6px 12px', background: 'transparent', color: 'var(--accent-cyan)', border: '1px solid var(--accent-cyan)', borderRadius: '4px', cursor: 'pointer', marginLeft: '12px' }}
                        >
                            <Map size={14} /> VIEW SIMULATION
                        </button>
                    </div>
                )}
                
                <div style={{ display: 'flex', alignItems: 'center', gap: '12px', marginLeft: '12px', paddingLeft: '12px', borderLeft: '1px solid rgba(255,255,255,0.2)' }}>
                    <button
                        onClick={onToggleView}
                        style={{
                            background: currentView === 'DATABASE' ? 'var(--accent-cyan)' : 'transparent',
                            color: currentView === 'DATABASE' ? 'black' : 'var(--accent-cyan)',
                            border: '1px solid var(--accent-cyan)',
                            padding: '6px 12px',
                            borderRadius: '4px',
                            cursor: 'pointer',
                            fontWeight: 'bold',
                            display: 'flex',
                            alignItems: 'center',
                            gap: '6px'
                        }}
                    >
                        <Activity size={14} /> {currentView === 'DATABASE' ? 'BACK TO MISSION' : 'RESULTS DATABASE'}
                    </button>
                    {isLive ? (
                        <div className="live-indicator">
                            <div className="live-dot" />
                            <Radio size={14} /> LIVE
                        </div>
                    ) : (
                        <div className="offline-indicator">
                            <div className="offline-dot" />
                            <WifiOff size={14} /> OFFLINE
                        </div>
                    )}
                </div>
            </div>
        </header>
    );
}
