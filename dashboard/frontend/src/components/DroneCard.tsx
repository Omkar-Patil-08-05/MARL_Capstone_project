import React from 'react';
import { Navigation, AlertTriangle, CheckCircle2, Camera, Gauge } from 'lucide-react';
import type { DroneTelemetry } from '../types/telemetry';

const DRONE_PALETTE = ['#00f2fe', '#4facfe', '#a78bfa', '#f472b6', '#fb923c', '#facc15'];

export function DroneCard({ drone, activeView, onToggleCamera }: { drone: DroneTelemetry, activeView?: 'MAP'|'CAMERA', onToggleCamera?: () => void }) {
    const isSafe = !drone.safety_override;
    const isHover = drone.action === 'Hover';
    const idx = parseInt(drone.id.replace('drone_', ''), 10);
    const color = DRONE_PALETTE[idx % DRONE_PALETTE.length];

    // Live speed from actual PX4 odometry
    const hasSpeed = drone.speed !== undefined && drone.speed !== null;
    const speedStr = hasSpeed ? `${drone.speed!.toFixed(2)} m/s` : '---';
    
    return (
        <div className="panel flex-col gap-4">
            <div className="flex-row justify-between items-center" style={{ borderBottom: '1px solid var(--border-light)', paddingBottom: '12px', gap: '8px' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '12px', flexShrink: 0 }}>
                    <h3 className="text-xl" style={{ margin: 0, color, display: 'flex', alignItems: 'center', gap: '8px' }}>
                        <div style={{ width: 10, height: 10, borderRadius: '50%', background: color, boxShadow: `0 0 6px ${color}` }} />
                        {drone.id.replace('_', ' ').toUpperCase()}
                    </h3>
                    <button 
                        onClick={onToggleCamera}
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
                            fontSize: '0.75rem'
                        }}
                    >
                        <Camera size={14} />
                    </button>
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

            {activeView === 'CAMERA' ? (
                <div style={{ marginTop: '8px', borderRadius: '4px', overflow: 'hidden', height: '120px', background: '#000', display: 'flex', justifyContent: 'center' }}>
                    <img 
                        src={`http://localhost:8000/api/camera/stream?drone_id=${idx}`} 
                        alt={`Drone ${idx} Camera`} 
                        style={{ height: '100%', objectFit: 'contain' }}
                        onError={(e) => { e.currentTarget.style.display = 'none'; }}
                    />
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
