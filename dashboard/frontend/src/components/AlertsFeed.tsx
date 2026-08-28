import React from 'react';
import { Bell, Info, AlertTriangle, CheckCircle } from 'lucide-react';
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
            <div className="panel-header">
                <Bell size={16} /> MISSION EVENTS
            </div>
            
            <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', overflowY: 'auto' }}>
                {alerts.length === 0 ? (
                    <div style={{ color: 'var(--text-muted)', fontSize: '0.85rem', textAlign: 'center', marginTop: '20px' }}>
                        No events recorded yet.
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
