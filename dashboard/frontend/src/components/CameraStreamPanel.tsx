import type { DroneTelemetry } from '../types/telemetry';

export function CameraStreamPanel({ drones }: { drones: DroneTelemetry[] }) {
    if (!drones || drones.length === 0) return null;

    return (
        <div className="panel" style={{ marginTop: '1rem' }}>
            <h3>LIVE CAMERA FEED</h3>
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '1rem' }}>
                {drones.map(drone => (
                    <div key={drone.id} style={{
                        background: 'black',
                        borderRadius: '0.5rem',
                        overflow: 'hidden',
                        position: 'relative',
                        aspectRatio: '4/3',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        border: '1px solid rgba(255,255,255,0.1)'
                    }}>
                        {/* Header for the camera */}
                        <div style={{
                            position: 'absolute',
                            top: 0, left: 0, right: 0,
                            background: 'rgba(0,0,0,0.7)',
                            padding: '0.25rem 0.5rem',
                            display: 'flex',
                            justifyContent: 'space-between',
                            zIndex: 10
                        }}>
                            <span style={{ fontSize: '0.75rem', fontWeight: 'bold', color: 'var(--text-main)' }}>
                                D{drone.id} CAMERA
                            </span>
                            <span style={{ fontSize: '0.75rem', color: 'var(--accent-red)' }}>
                                ● REC
                            </span>
                        </div>
                        
                        <img 
                            src={`http://localhost:8000/api/camera/stream?drone_id=${drone.id.toString().replace('drone_', '')}`} 
                            alt={`Drone ${drone.id} Camera Stream`}
                            style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                            onError={(e) => {
                                e.currentTarget.style.display = 'none';
                                e.currentTarget.nextElementSibling!.setAttribute('style', 'display: block; text-align: center; color: var(--text-muted);');
                            }}
                        />
                        <div style={{ display: 'none' }}>
                            <div className="spinner" style={{ 
                                width: '24px', height: '24px', 
                                border: '2px solid rgba(255,255,255,0.1)', 
                                borderTopColor: 'var(--accent-cyan)', 
                                borderRadius: '50%', 
                                margin: '0 auto 0.5rem auto' 
                            }} />
                            <div style={{ fontSize: '0.8rem', letterSpacing: '1px' }}>WAITING FOR STREAM</div>
                        </div>
                    </div>
                ))}
            </div>
        </div>
    );
}
