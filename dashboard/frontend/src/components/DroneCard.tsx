import React, { useState, useEffect } from 'react';
import { Navigation, AlertTriangle, CheckCircle2, Camera, RefreshCw, Zap } from 'lucide-react';
import type { DroneTelemetry } from '../types/telemetry';

const DRONE_PALETTE = ['#00f2fe', '#4facfe', '#a78bfa', '#f472b6', '#fb923c', '#facc15'];

export function DroneCard({
    drone,
    activeView,
    onToggleCamera,
    cameraEnabled = false
}: {
    drone: DroneTelemetry;
    activeView?: 'MAP' | 'CAMERA';
    onToggleCamera?: () => void;
    cameraEnabled?: boolean;
}) {
    const [imgFailed, setImgFailed] = useState(false);
    const [key, setKey] = useState(0);

    const isSafe = !drone.safety_override;
    const isHover = drone.action === 'Hover';
    const idx = parseInt(drone.id.replace('drone_', ''), 10);
    const color = DRONE_PALETTE[idx % DRONE_PALETTE.length];

    // Live speed from actual PX4 odometry
    const hasSpeed = drone.speed !== undefined && drone.speed !== null;
    const speedStr = hasSpeed ? `${drone.speed!.toFixed(2)} m/s` : '---';

    // Stream URL proxied through Vite to FastAPI backend on port 8000
    const streamUrl = `/api/camera/stream?drone_id=${idx}`;

    // Auto-reconnect watchdog: if the stream drops or has not yet started, retry periodically
    useEffect(() => {
        let retryTimer: ReturnType<typeof setTimeout> | null = null;
        if (cameraEnabled && imgFailed && activeView === 'CAMERA') {
            retryTimer = setTimeout(() => {
                setImgFailed(false);
                setKey(k => k + 1);
            }, 2500);
        }
        return () => {
            if (retryTimer) clearTimeout(retryTimer);
        };
    }, [imgFailed, activeView, cameraEnabled]);

    return (
        <div className="panel flex-col gap-4">
            <div className="flex-row justify-between items-center" style={{ borderBottom: '1px solid var(--border-light)', paddingBottom: '12px', gap: '8px' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '12px', flexShrink: 0 }}>
                    <h3 className="text-xl" style={{ margin: 0, color, display: 'flex', alignItems: 'center', gap: '8px' }}>
                        <div style={{ width: 10, height: 10, borderRadius: '50%', background: color, boxShadow: `0 0 6px ${color}` }} />
                        {drone.id.replace('_', ' ').toUpperCase()}
                    </h3>
                    {cameraEnabled ? (
                        <button 
                            onClick={onToggleCamera}
                            title={activeView === 'CAMERA' ? "Switch to Telemetry" : "Switch to Live Camera"}
                            style={{
                                background: activeView === 'CAMERA' ? color : 'transparent',
                                color: activeView === 'CAMERA' ? '#000' : 'var(--text-secondary)',
                                border: `1px solid ${color}`,
                                borderRadius: '4px',
                                padding: '4px 8px',
                                cursor: 'pointer',
                                display: 'flex',
                                alignItems: 'center',
                                gap: '4px',
                                fontSize: '0.75rem',
                                fontWeight: 600
                            }}
                        >
                            <Camera size={14} />
                            CAM
                        </button>
                    ) : (
                        <span style={{
                            fontSize: '0.7rem',
                            fontFamily: 'var(--font-mono)',
                            padding: '3px 7px',
                            borderRadius: '4px',
                            background: 'rgba(0, 242, 254, 0.08)',
                            color: 'var(--accent-cyan)',
                            border: '1px solid rgba(0, 242, 254, 0.2)',
                            display: 'flex',
                            alignItems: 'center',
                            gap: '4px'
                        }}>
                            <Zap size={11} /> HIGH-SPEED
                        </span>
                    )}
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px', flexShrink: 0 }}>
                    <span style={{ 
                        fontSize: '0.75rem', 
                        padding: '4px 8px', 
                        borderRadius: '4px',
                        background: 'rgba(255,255,255,0.1)',
                        fontFamily: 'var(--font-mono)',
                        whiteSpace: 'nowrap',
                        textAlign: 'right'
                    }}>
                        {drone.state}
                    </span>
                </div>
            </div>

            {cameraEnabled && activeView === 'CAMERA' ? (
                <div style={{ 
                    position: 'relative', 
                    marginTop: '8px', 
                    borderRadius: '6px', 
                    overflow: 'hidden', 
                    height: '140px', 
                    background: '#0d1117', 
                    display: 'flex', 
                    justifyContent: 'center', 
                    alignItems: 'center',
                    border: '1px solid rgba(255,255,255,0.1)'
                }}>
                    {!imgFailed ? (
                        <>
                            <img 
                                key={key}
                                src={streamUrl} 
                                alt={`Drone ${idx} Camera Stream`} 
                                style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                                onLoad={() => setImgFailed(false)}
                                onError={() => setImgFailed(true)}
                            />
                            <div style={{
                                position: 'absolute',
                                top: '6px',
                                left: '8px',
                                background: 'rgba(0,0,0,0.65)',
                                padding: '2px 6px',
                                borderRadius: '4px',
                                fontSize: '0.65rem',
                                fontFamily: 'var(--font-mono)',
                                color: color,
                                display: 'flex',
                                alignItems: 'center',
                                gap: '4px'
                            }}>
                                <span style={{ width: 6, height: 6, borderRadius: '50%', background: '#00ff87', display: 'inline-block' }} />
                                LIVE FEED - D{idx}
                            </div>
                        </>
                    ) : (
                        <div style={{
                            display: 'flex',
                            flexDirection: 'column',
                            alignItems: 'center',
                            justifyContent: 'center',
                            gap: '8px',
                            color: 'var(--text-muted)',
                            padding: '12px',
                            textAlign: 'center'
                        }}>
                            <Camera size={24} style={{ opacity: 0.5 }} />
                            <div style={{ fontSize: '0.75rem', fontFamily: 'var(--font-mono)', color: 'var(--accent-orange)' }}>
                                CAMERA SIGNAL CONNECTING
                            </div>
                            <button
                                onClick={() => { setImgFailed(false); setKey(k => k + 1); }}
                                style={{
                                    background: 'rgba(255,255,255,0.08)',
                                    border: '1px solid rgba(255,255,255,0.2)',
                                    color: 'var(--text-main)',
                                    padding: '3px 8px',
                                    borderRadius: '4px',
                                    cursor: 'pointer',
                                    fontSize: '0.7rem',
                                    display: 'flex',
                                    alignItems: 'center',
                                    gap: '4px'
                                }}
                            >
                                <RefreshCw size={10} /> RETRY FEED
                            </button>
                        </div>
                    )}
                </div>
            ) : (
                <div className="flex-row justify-between">
                    <div className="value-display">
                        <span className="value-label">World Position (m)</span>
                        <span className="value-text">
                            X: {drone.x.toFixed(1)} <br/>
                            Y: {drone.y.toFixed(1)} <br/>
                            Alt: {drone.z.toFixed(1)}
                        </span>
                    </div>
                    
                    <div className="value-display" style={{ alignItems: 'flex-end' }}>
                        <span className="value-label">Grid Cell</span>
                        <span className="value-text">({drone.grid_x}, {drone.grid_y})</span>
                        <span className="value-label" style={{ marginTop: '8px' }}>Speed</span>
                        <span className="value-text" style={{ color: hasSpeed && drone.speed! > 1.0 ? '#00f2fe' : 'inherit' }}>{speedStr}</span>
                    </div>
                </div>
            )}

            <div className="flex-row justify-between items-center mt-4">
                <div className="flex-row items-center gap-2">
                    <Navigation size={18} style={{ color: isHover ? 'var(--accent-yellow)' : color }} />
                    <span style={{ fontFamily: 'var(--font-mono)', fontWeight: 600 }}>
                        Action: {drone.action}
                    </span>
                </div>
                
                <div className="flex-row items-center gap-2" style={{ color: isSafe ? 'var(--accent-green)' : 'var(--accent-red)' }}>
                    {isSafe ? (
                        <><CheckCircle2 size={16} /> <span style={{ fontSize: '0.85rem' }}>NORMAL</span></>
                    ) : (
                        <><AlertTriangle size={16} /> <span style={{ fontSize: '0.85rem' }}>SAFETY OVERRIDE</span></>
                    )}
                </div>
            </div>
        </div>
    );
}
