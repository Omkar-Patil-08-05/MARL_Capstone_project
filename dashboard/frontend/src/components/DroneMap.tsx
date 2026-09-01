import React, { useMemo } from 'react';
import type { WorldData, DroneTelemetry, ExploredCell, VictimState } from '../types/telemetry';
import { Map } from 'lucide-react';

const DRONE_PALETTE = ['#00f2fe', '#4facfe', '#a78bfa', '#f472b6', '#fb923c', '#facc15'];

interface DroneMapProps {
    worldData: WorldData | null;
    drones: DroneTelemetry[];
    history: Record<string, {x: number, y: number}[]>;
    exploredCells: ExploredCell[];
    victims: VictimState[];
    trackedVictims: VictimState[];
    activeFrontiers?: Record<string, [number, number]>;
}

export function DroneMap({ worldData, drones, history, exploredCells, victims, trackedVictims, activeFrontiers }: DroneMapProps) {
    const [isExpanded, setIsExpanded] = React.useState(false);

    // If modal is active, listen for escape key
    React.useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.key === 'Escape') setIsExpanded(false);
        };
        if (isExpanded) window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [isExpanded]);

    const gridLines = useMemo(() => {
        if (!worldData) return [];
        const { grid, world } = worldData;
        const mpc = grid.meters_per_cell;
        const lines = [];
        for (let i = 0; i <= grid.width; i++) {
            const x = i * mpc;
            lines.push(<line key={`vx-${i}`} x1={x} y1={0} x2={x} y2={world.height_m} stroke="rgba(255,255,255,0.02)" strokeWidth={0.2} />);
        }
        for (let i = 0; i <= grid.height; i++) {
            const y = i * mpc;
            lines.push(<line key={`vy-${i}`} x1={0} y1={y} x2={world.width_m} y2={y} stroke="rgba(255,255,255,0.02)" strokeWidth={0.2} />);
        }
        return lines;
    }, [worldData]);

    const exploredRects = useMemo(() => {
        if (!worldData) return [];
        const mpc = worldData.grid.meters_per_cell;
        return exploredCells.map((c) => (
            <rect
                key={`exp-${c.x}-${c.y}`}
                x={c.x * mpc}
                y={c.y * mpc}
                width={mpc}
                height={mpc}
                fill="rgba(0, 242, 254, 0.6)"
                stroke="rgba(0, 242, 254, 0.8)"
                strokeWidth={0.2}
            />
        ));
    }, [worldData, exploredCells]);

    if (!worldData) {
        return (
            <div className="panel flex-col items-center" style={{ flex: 1, justifyContent: 'center' }}>
                <Map size={48} className="text-muted" style={{ marginBottom: '16px' }} />
                <span className="text-muted">Waiting for world data...</span>
            </div>
        );
    }

    const { grid, world, obstacles } = worldData;
    const viewBox = `0 0 ${world.width_m} ${world.height_m}`;
    const mpc = grid.meters_per_cell;

    const getDroneColor = (droneId: string): string => {
        const idx = parseInt(droneId.replace('drone_', ''), 10);
        return DRONE_PALETTE[idx % DRONE_PALETTE.length];
    };

    // The inner map rendering is identical for both views
    const renderMapSvg = () => (
        <svg
            viewBox={viewBox}
            style={{
                width: '100%',
                height: '100%',
                maxHeight: '100%',
                maxWidth: '100%'
            }}
        >
            <rect x="0" y="0" width={world.width_m} height={world.height_m} fill="#090a0f" stroke="rgba(255,255,255,0.1)" strokeWidth="1" rx="4" />
            <g className="grid-layer">{gridLines}</g>
            <g className="explored-layer">{exploredRects}</g>

            <g className="obstacles-layer">
                {obstacles.map(obs => {
                    const w = obs.aabb.max_x - obs.aabb.min_x;
                    const h = obs.aabb.max_y - obs.aabb.min_y;
                    return (
                        <rect
                            key={obs.id}
                            x={obs.aabb.min_x - world.origin_x}
                            y={obs.aabb.min_y - world.origin_y}
                            width={w}
                            height={h}
                            fill="rgba(255, 255, 255, 0.15)"
                            stroke="rgba(255, 255, 255, 0.3)"
                            strokeWidth={0.5}
                        />
                    );
                })}
            </g>

            <g className="victims-gt-layer">
                {victims.map(v => {
                    const cx = (v.x * mpc) + (mpc / 2);
                    const cy = (v.y * mpc) + (mpc / 2);
                    return (
                        <g key={`gt-${v.id}`} transform={`translate(${cx}, ${cy})`}>
                            <circle r={1.5} fill="rgba(255,255,255,0.05)" stroke="rgba(255,255,255,0.1)" strokeWidth={0.5} />
                        </g>
                    );
                })}
            </g>

            <g className="victims-layer">
                {trackedVictims && trackedVictims.map(v => {
                    const cx = (v.x * mpc) + (mpc / 2);
                    const cy = (v.y * mpc) + (mpc / 2);
                    return (
                        <g key={v.id} transform={`translate(${cx}, ${cy})`}>
                            <circle r={2.5} fill="rgba(0,255,135,0.25)" stroke="#00ff87" strokeWidth={0.6} />
                            <circle r={1.2} fill="#00ff87" />
                            <text y={0.6} fill="#000" fontSize={1.6} fontWeight={900} textAnchor="middle" style={{ fontFamily: 'sans-serif' }}>✓</text>
                            <text y={-4} fill="#00ff87" fontSize={2.2} fontWeight={700} textAnchor="middle" style={{ fontFamily: 'var(--font-mono)' }}>{v.id}</text>
                        </g>
                    );
                })}
            </g>

            <g className="trajectories-layer">
                {Object.entries(history).map(([droneId, pts]) => {
                    if (pts.length < 2) return null;
                    const color = getDroneColor(droneId);
                    const pathData = `M ${pts[0].x} ${pts[0].y} ` + pts.slice(1).map(p => `L ${p.x} ${p.y}`).join(' ');
                    return (
                        <path
                            key={`path-${droneId}`}
                            d={pathData}
                            fill="none"
                            stroke={color}
                            strokeWidth={0.8}
                            opacity={0.5}
                        />
                    );
                })}
            </g>



            <g className="drones-layer">
                {drones.map(d => {
                    const color = getDroneColor(d.id);
                    return (
                        <g key={d.id} transform={`translate(${d.x}, ${d.y})`} style={{ transition: 'transform 0.5s linear' }}>
                            <circle r={8} fill={color} opacity={0.08} />
                            <circle r={2.5} fill={color} stroke="#fff" strokeWidth={0.5} />
                            <text
                                y={-5} fill="#fff" fontSize={3} textAnchor="middle" fontFamily="var(--font-mono)" fontWeight={700}
                                style={{ textShadow: '0px 1px 3px rgba(0,0,0,0.9)' }}
                            >
                                {d.id.replace('drone_', 'D')}
                            </text>
                        </g>
                    );
                })}
            </g>
        </svg>
    );

    const renderLegend = () => (
        <div style={{
            position: 'absolute', bottom: '12px', left: '12px', zIndex: 10,
            background: 'rgba(18, 22, 28, 0.9)', padding: '8px 12px', borderRadius: '6px',
            display: 'flex', flexDirection: 'column', gap: '4px', fontSize: '0.65rem', letterSpacing: '0.5px'
        }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, background: 'rgba(0,242,254,0.6)', border: '1px solid rgba(0,242,254,0.8)' }} />
                <span>SEARCHED</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, background: 'rgba(255,255,255,0.15)', border: '1px solid rgba(255,255,255,0.3)' }} />
                <span>OBSTACLE</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, borderRadius: '50%', background: '#ff5533' }} />
                <span>VICTIM</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, borderRadius: '50%', background: '#00ff87' }} />
                <span>DETECTED</span>
            </div>
            {drones.map(d => (
                <div key={d.id} style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                    <div style={{ width: 10, height: 10, borderRadius: '50%', background: getDroneColor(d.id) }} />
                    <span>{d.id.replace('drone_', 'DRONE ')}</span>
                </div>
            ))}
        </div>
    );

    return (
        <>
            <div className="panel flex-col" style={{ flex: 1, position: 'relative', overflow: 'hidden' }}>
                <div style={{ position: 'absolute', top: '12px', left: '12px', right: '12px', zIndex: 10, display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start' }}>
                    <div style={{ background: 'rgba(18, 22, 28, 0.85)', padding: '4px 10px', borderRadius: '4px', display: 'flex', alignItems: 'center', gap: '6px', fontSize: '0.75rem', letterSpacing: '1px', color: 'var(--text-muted)' }}>
                        <Map size={14} /> LIVE SAR MAP
                    </div>
                    <button
                        onClick={() => setIsExpanded(true)}
                        style={{ background: 'rgba(0, 242, 254, 0.1)', border: '1px solid var(--accent-cyan)', padding: '4px 10px', borderRadius: '4px', color: 'var(--accent-cyan)', fontSize: '0.75rem', cursor: 'pointer', fontWeight: 'bold' }}
                    >
                        VIEW FULL MAP
                    </button>
                </div>

                {renderLegend()}

                <div style={{ flex: 1, width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '24px', boxSizing: 'border-box' }}>
                    {renderMapSvg()}
                </div>
            </div>

            {isExpanded && (
                <div style={{
                    position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
                    background: 'rgba(0,0,0,0.9)', zIndex: 9999,
                    display: 'flex', flexDirection: 'column', padding: '2rem'
                }}>
                    <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
                        <h2 style={{ margin: 0, color: 'var(--accent-cyan)', letterSpacing: '2px', display: 'flex', alignItems: 'center', gap: '8px' }}>
                            <Map size={24} /> SAR COVERAGE HEATMAP
                        </h2>
                        <button
                            onClick={() => setIsExpanded(false)}
                            style={{ background: 'transparent', border: '1px solid var(--text-muted)', padding: '6px 16px', borderRadius: '4px', color: 'var(--text-main)', cursor: 'pointer', fontSize: '0.9rem' }}
                        >
                            CLOSE X
                        </button>
                    </div>

                    <div style={{ flex: 1, position: 'relative', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px', background: '#000', overflow: 'hidden' }}>
                        {renderLegend()}
                        <div style={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '2rem', boxSizing: 'border-box' }}>
                            {renderMapSvg()}
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}
