import React from 'react';
import { Navigation, AlertTriangle, CheckCircle2 } from 'lucide-react';
import type { DroneTelemetry } from '../types/telemetry';

const DRONE_PALETTE = ['#00f2fe', '#4facfe', '#a78bfa', '#f472b6', '#fb923c', '#facc15'];

export function DroneCard({ drone }: { drone: DroneTelemetry }) {
    const isSafe = !drone.safety_override;
    const isHover = drone.action === 'Hover';
    const idx = parseInt(drone.id.replace('drone_', ''), 10);
    const color = DRONE_PALETTE[idx % DRONE_PALETTE.length];
    
    return (
        <div className="panel flex-col gap-4">
            <div className="flex-row justify-between items-center" style={{ borderBottom: '1px solid var(--border-light)', paddingBottom: '12px' }}>
                <h3 className="text-xl" style={{ margin: 0, color, display: 'flex', alignItems: 'center', gap: '8px' }}>
                    <div style={{ width: 10, height: 10, borderRadius: '50%', background: color, boxShadow: `0 0 6px ${color}` }} />
                    {drone.id.replace('_', ' ').toUpperCase()}
                </h3>
                <span style={{ 
                    fontSize: '0.75rem', 
                    padding: '4px 8px', 
                    borderRadius: '4px',
                    background: 'rgba(255,255,255,0.1)',
                    fontFamily: 'var(--font-mono)'
                }}>
                    {drone.state}
                </span>
            </div>

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
                </div>
            </div>

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
