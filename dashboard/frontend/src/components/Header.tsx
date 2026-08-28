import React from 'react';
import { Activity, Radio, Wifi, WifiOff, Square, RefreshCcw } from 'lucide-react';
import type { MissionTelemetry, BackendMissionStatus } from '../types/telemetry';

interface HeaderProps {
    isConnected: boolean;
    telemetry: MissionTelemetry | null;
    backendStatus: BackendMissionStatus;
    onStop: () => void;
    onReset: () => void;
}

export function Header({ isConnected, telemetry, backendStatus, onStop, onReset }: HeaderProps) {
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
                <div className="value-display" style={{ alignItems: 'flex-end', marginRight: '24px' }}>
                    <span className="value-label">Backend Status</span>
                    <span className="value-text text-cyan">{backendStatus.state}</span>
                </div>
                
                {telemetry && (
                    <div className="value-display" style={{ alignItems: 'flex-end', marginRight: '24px' }}>
                        <span className="value-label">Mission Status</span>
                        <span className="value-text text-cyan">{telemetry.status}</span>
                    </div>
                )}
                
                {backendStatus.state !== 'IDLE' && (
                    <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
                        <button 
                            className="btn-danger" 
                            onClick={() => { if(window.confirm('Are you sure you want to STOP the mission?')) onStop() }}
                            disabled={backendStatus.state === 'STOPPING'}
                            style={{ display: 'flex', alignItems: 'center', gap: '4px', padding: '6px 12px', background: '#dc2626', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                        >
                            <Square size={14} /> STOP
                        </button>
                        <button 
                            className="btn-warning" 
                            onClick={() => { if(window.confirm('Reset mission?')) onReset() }}
                            style={{ display: 'flex', alignItems: 'center', gap: '4px', padding: '6px 12px', background: '#d97706', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                        >
                            <RefreshCcw size={14} /> RESET
                        </button>
                    </div>
                )}
                <div style={{ display: 'flex', alignItems: 'center' }}>
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
