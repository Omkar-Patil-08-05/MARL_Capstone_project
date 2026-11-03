import React from 'react';
import { Bell, Info, AlertTriangle, CheckCircle, Activity } from 'lucide-react';
import type { AlertEvent } from '../types/telemetry';

export function AlertsFeed({ alerts }: { alerts: AlertEvent[] }) {
    
    const getIcon = (type: string) => {
        switch(type) {
            case 'warning': return <AlertTriangle size={16} className="text-red" />;
            case 'success': return <CheckCircle size={16} className="text-green" />;
            default: return <Info size={16} className="text-cyan" />;
        }
    };

    return (
        <div className="panel flex-col" style={{ flex: 1, minHeight: '200px' }}>
            <div className="panel-header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                    <Bell size={16} /> MISSION EVENTS
                </div>
                {alerts.length > 0 && (
                    <span style={{ fontSize: '0.75rem', fontFamily: 'var(--font-mono)', color: 'var(--accent-cyan)' }}>
                        {alerts.length} logged
                    </span>
                )}
            </div>
            
            <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', overflowY: 'auto' }}>
                {alerts.length === 0 ? (
                    <div style={{
                        display: 'flex',
                        flexDirection: 'column',
                        alignItems: 'center',
                        justifyContent: 'center',
                        padding: '1.5rem 1rem',
                        textAlign: 'center',
                        background: 'rgba(255,255,255,0.02)',
                        borderRadius: '6px',
                        border: '1px dashed rgba(255,255,255,0.1)',
                        margin: '8px 0'
                    }}>
                        <Activity size={22} style={{ color: 'var(--accent-cyan)', opacity: 0.6, marginBottom: '8px' }} />
                        <span style={{ fontSize: '0.85rem', fontWeight: 600, color: 'var(--text-main)', marginBottom: '4px' }}>
                            No mission events recorded
                        </span>
                        <span style={{ fontSize: '0.75rem', color: 'var(--text-muted)', lineHeight: 1.4, maxWidth: '240px' }}>
                            Live events (autonomous detections, state transitions, safety interventions) appear here as the swarm navigates.
                        </span>
                    </div>
                ) : (
                    alerts.map((alert, i) => {
                        const d = new Date(alert.timestamp * 1000);
                        const timeStr = `${d.getHours().toString().padStart(2, '0')}:${d.getMinutes().toString().padStart(2, '0')}:${d.getSeconds().toString().padStart(2, '0')}`;
                        
                        return (
                            <div key={alert.id} style={{ 
                                display: 'flex', 
                                gap: '12px', 
                                alignItems: 'flex-start',
                                padding: '8px',
                                background: 'rgba(255,255,255,0.03)',
                                borderRadius: '6px',
                                borderLeft: `2px solid ${alert.type === 'warning' ? 'var(--accent-red)' : alert.type === 'success' ? 'var(--accent-green)' : 'var(--accent-cyan)'}`,
                                opacity: Math.max(0.3, 1 - (i * 0.1)) // fade older events
                            }}>
                                <div style={{ marginTop: '2px' }}>
                                    {getIcon(alert.type)}
                                </div>
                                <div className="flex-col">
                                    <span style={{ fontSize: '0.85rem', color: 'var(--text-main)' }}>{alert.message}</span>
                                    <span style={{ fontSize: '0.7rem', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>{timeStr}</span>
                                </div>
                            </div>
                        );
                    })
                )}
            </div>
        </div>
    );
}
