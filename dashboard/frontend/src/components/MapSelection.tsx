import React, { useState } from 'react';
import type { MapRegistry, BackendMissionStatus } from '../types/telemetry';
import { Map, Play, Loader, AlertTriangle, RotateCcw, Cpu, CheckCircle2, Users } from 'lucide-react';

interface MapSelectionProps {
    registry: MapRegistry;
    backendStatus: BackendMissionStatus;
    onStart: (mapId: string, droneCount: number, victimCount: number) => void;
}

const DRONE_PALETTE = ['#00f2fe', '#4facfe', '#a78bfa', '#f472b6', '#fb923c', '#facc15', '#4ade80', '#2dd4bf'];

export function MapSelection({ registry, backendStatus, onStart }: MapSelectionProps) {
    const maps = Object.values(registry);
    const [selectedMapId, setSelectedMapId] = useState<string>(maps.length > 0 ? maps[0].id : '');
    const [droneCount, setDroneCount] = useState<number>(6);
    const [victimCount, setVictimCount] = useState<number>(5);

    const selectedMap = registry[selectedMapId];
    const isStarting = ['STARTING', 'SIMULATOR_READY', 'QMIX_STARTING'].includes(backendStatus.state);
    const isRunning = backendStatus.state === 'RUNNING';
    const isError = backendStatus.state === 'ERROR';

    // Map compatibility check (QMIX + Deterministic mixed architecture)
    const policyCompatible = (selectedMap?.policy_compatible ?? false);
    const canStart = policyCompatible && !isStarting && !isRunning;

    const getButtonLabel = () => {
        if (isStarting) {
            if (backendStatus.state === 'STARTING') return 'STARTING SIMULATION...';
            if (backendStatus.state === 'SIMULATOR_READY') return 'WAITING FOR EKF2...';
            if (backendStatus.state === 'QMIX_STARTING') return 'STARTING QMIX...';
        }
        if (isRunning) return 'MISSION ACTIVE';
        if (isError) return 'RETRY MISSION';
        return 'START MISSION';
    };

    const getButtonIcon = () => {
        if (isStarting) return <Loader size={20} style={{ animation: 'spin 1s linear infinite' }} />;
        if (isError) return <RotateCcw size={20} />;
        return <Play size={20} />;
    };

    return (
        <div style={{ padding: '2rem', maxWidth: '700px', margin: '0 auto', display: 'flex', flexDirection: 'column', gap: '20px' }}>
            <div className="panel flex-col gap-4" style={{ padding: '24px' }}>
                {/* Title */}
                <div style={{ textAlign: 'center', marginBottom: '8px' }}>
                    <h2 className="text-xl" style={{ margin: '0 0 4px 0', color: 'var(--accent-cyan)', display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '10px' }}>
                        <Map size={22} /> SELECT SEARCH MISSION
                    </h2>
                    <span className="text-muted" style={{ fontSize: '0.75rem', letterSpacing: '1.5px' }}>MARL SWARM COMMAND CENTER</span>
                </div>

                {/* Map Cards */}
                <div style={{ display: 'flex', gap: '12px' }}>
                    {maps.map(m => {
                        const isSelected = selectedMapId === m.id;
                        return (
                            <div
                                key={m.id}
                                onClick={() => setSelectedMapId(m.id)}
                                style={{
                                    flex: 1,
                                    cursor: 'pointer',
                                    padding: '16px',
                                    borderRadius: '8px',
                                    border: isSelected ? '2px solid var(--accent-cyan)' : '1px solid var(--border-light)',
                                    background: isSelected ? 'rgba(0, 242, 254, 0.08)' : 'rgba(255,255,255,0.03)',
                                    transition: 'all 0.2s',
                                    position: 'relative'
                                }}
                            >
                                {isSelected && (
                                    <div style={{
                                        position: 'absolute', top: '8px', right: '8px',
                                        background: 'var(--accent-cyan)', color: '#000',
                                        fontSize: '0.6rem', fontWeight: 700, padding: '2px 6px',
                                        borderRadius: '3px', letterSpacing: '0.5px'
                                    }}>SELECTED</div>
                                )}
                                <div style={{ fontSize: '1rem', fontWeight: 600, marginBottom: '4px' }}>{m.name}</div>
                                <div className="text-muted" style={{ fontSize: '0.8rem' }}>{m.grid_width}×{m.grid_height} Grid</div>
                            </div>
                        );
                    })}
                </div>

                {/* Map Details */}
                {selectedMap && (
                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '12px', padding: '12px', background: 'rgba(255,255,255,0.02)', borderRadius: '8px' }}>
                        <div className="value-display">
                            <span className="value-label">Dimensions</span>
                            <span className="value-text" style={{ fontSize: '1rem' }}>{selectedMap.grid_width} × {selectedMap.grid_height}</span>
                        </div>
                        <div className="value-display">
                            <span className="value-label">Physical Area</span>
                            <span className="value-text" style={{ fontSize: '1rem' }}>
                                {selectedMap.grid_width * selectedMap.meters_per_cell}m × {selectedMap.grid_height * selectedMap.meters_per_cell}m
                            </span>
                        </div>
                        <div className="value-display">
                            <span className="value-label">Victims</span>
                            <span className="value-text" style={{ fontSize: '1rem', color: '#ff5533' }}>{victimCount}</span>
                        </div>
                    </div>
                )}

                {/* Drone Count Selector */}
                <div>
                    <div className="value-label" style={{ marginBottom: '8px' }}>NUMBER OF DRONES</div>
                    <div style={{ display: 'flex', gap: '8px' }}>
                        {[1, 2, 3, 4, 5, 6].map(n => {
                            const isActive = droneCount === n;
                            return (
                                <button
                                    key={n}
                                    onClick={() => setDroneCount(n)}
                                    style={{
                                        flex: 1,
                                        padding: '10px 0',
                                        border: isActive ? '2px solid var(--accent-cyan)' : '1px solid var(--border-light)',
                                        borderRadius: '6px',
                                        background: isActive ? 'rgba(0, 242, 254, 0.12)' : 'rgba(255,255,255,0.03)',
                                        color: isActive ? 'var(--accent-cyan)' : 'var(--text-main)',
                                        cursor: 'pointer',
                                        fontFamily: 'var(--font-mono)',
                                        fontWeight: isActive ? 700 : 400,
                                        fontSize: '1rem',
                                        transition: 'all 0.15s',
                                        position: 'relative'
                                    }}
                                >
                                    {n}
                                </button>
                            );
                        })}
                    </div>
                </div>

                {/* Victim Count Selector */}
                <div>
                    <div className="value-label" style={{ marginBottom: '8px', display: 'flex', alignItems: 'center', gap: '6px' }}>
                        <Users size={14} /> NUMBER OF VICTIMS
                    </div>
                    <div style={{ display: 'flex', gap: '6px' }}>
                        {[1, 2, 3, 4, 5, 6, 7, 8, 9, 10].map(n => {
                            const isActive = victimCount === n;
                            return (
                                <button
                                    key={n}
                                    onClick={() => setVictimCount(n)}
                                    style={{
                                        flex: 1,
                                        padding: '10px 0',
                                        border: isActive ? '2px solid #ff5533' : '1px solid var(--border-light)',
                                        borderRadius: '6px',
                                        background: isActive ? 'rgba(255, 85, 51, 0.12)' : 'rgba(255,255,255,0.03)',
                                        color: isActive ? '#ff5533' : 'var(--text-main)',
                                        cursor: 'pointer',
                                        fontFamily: 'var(--font-mono)',
                                        fontWeight: isActive ? 700 : 400,
                                        fontSize: '0.9rem',
                                        transition: 'all 0.15s',
                                        position: 'relative'
                                    }}
                                >
                                    {n}
                                </button>
                            );
                        })}
                    </div>
                </div>

                {/* Policy & Compatibility */}
                <div style={{ display: 'flex', gap: '12px' }}>
                    <div style={{ flex: 1, display: 'flex', alignItems: 'center', gap: '8px', padding: '10px 14px', background: 'rgba(255,255,255,0.03)', borderRadius: '6px' }}>
                        <Cpu size={16} className="text-cyan" />
                        <div className="value-display">
                            <span className="value-label" style={{ fontSize: '0.65rem' }}>Policy</span>
                            <span style={{ fontFamily: 'var(--font-mono)', fontSize: '0.85rem', fontWeight: 600 }}>QMIX V4</span>
                        </div>
                    </div>
                    <div style={{
                        flex: 1, display: 'flex', alignItems: 'center', gap: '8px',
                        padding: '10px 14px', borderRadius: '6px',
                        background: policyCompatible ? 'rgba(0, 255, 135, 0.06)' : 'rgba(234, 179, 8, 0.06)',
                        border: `1px solid ${policyCompatible ? 'rgba(0,255,135,0.2)' : 'rgba(234,179,8,0.2)'}`
                    }}>
                        {policyCompatible
                            ? <><CheckCircle2 size={16} className="text-green" /><span style={{ fontSize: '0.85rem', fontWeight: 600, color: 'var(--accent-green)' }}>READY</span></>
                            : <><AlertTriangle size={16} style={{ color: 'var(--accent-yellow)' }} /><span style={{ fontSize: '0.75rem', fontWeight: 600, color: 'var(--accent-yellow)' }}>UNSUPPORTED MAP</span></>
                        }
                    </div>
                </div>

                {/* Drone Color Preview */}
                <div style={{ display: 'flex', gap: '8px', justifyContent: 'center' }}>
                    {Array.from({ length: droneCount }, (_, i) => (
                        <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '4px', padding: '4px 10px', background: 'rgba(255,255,255,0.04)', borderRadius: '4px' }}>
                            <div style={{ width: 8, height: 8, borderRadius: '50%', background: DRONE_PALETTE[i] }} />
                            <span style={{ fontSize: '0.75rem', fontFamily: 'var(--font-mono)' }}>D{i}</span>
                        </div>
                    ))}
                </div>

                {/* START BUTTON */}
                <button
                    disabled={!canStart}
                    onClick={() => canStart && onStart(selectedMapId, droneCount, victimCount)}
                    style={{
                        marginTop: '4px',
                        padding: '14px 24px',
                        fontSize: '1.1rem',
                        fontWeight: 700,
                        fontFamily: 'var(--font-mono)',
                        letterSpacing: '1px',
                        display: 'flex',
                        justifyContent: 'center',
                        alignItems: 'center',
                        gap: '10px',
                        background: canStart
                            ? 'linear-gradient(135deg, #00f2fe 0%, #4facfe 100%)'
                            : isStarting
                                ? 'linear-gradient(135deg, #1e3a5f 0%, #2d5a87 100%)'
                                : 'rgba(255,255,255,0.06)',
                        color: canStart ? '#000' : isStarting ? '#7cb3d9' : 'var(--text-muted)',
                        border: canStart ? '2px solid var(--accent-cyan)' : '1px solid var(--border-light)',
                        borderRadius: '8px',
                        cursor: canStart ? 'pointer' : 'not-allowed',
                        transition: 'all 0.2s',
                        boxShadow: canStart ? '0 0 20px rgba(0, 242, 254, 0.25)' : 'none',
                    }}
                >
                    {getButtonIcon()}
                    {getButtonLabel()}
                </button>
            </div>

            {/* CSS for spinner */}
            <style>{`
                @keyframes spin {
                    from { transform: rotate(0deg); }
                    to { transform: rotate(360deg); }
                }
            `}</style>
        </div>
    );
}
