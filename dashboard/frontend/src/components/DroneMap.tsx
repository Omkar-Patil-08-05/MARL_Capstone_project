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
    coverage?: number;
}

export function DroneMap({ worldData, drones, history, exploredCells, victims, trackedVictims, coverage = 0 }: DroneMapProps) {
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
            lines.push(<line key={`vx-${i}`} x1={x} y1={0} x2={x} y2={world.height_m} stroke="rgba(255,255,255,0.03)" strokeWidth={0.2} />);
        }
        for (let i = 0; i <= grid.height; i++) {
            const y = i * mpc;
            lines.push(<line key={`vy-${i}`} x1={0} y1={y} x2={world.width_m} y2={y} stroke="rgba(255,255,255,0.03)" strokeWidth={0.2} />);
        }
        return lines;
    }, [worldData]);

    const unsearchedRects = useMemo(() => {
        if (!worldData) return [];
        const { grid } = worldData;
        const mpc = grid.meters_per_cell;
        const rects = [];

        for (let x = 0; x < grid.width; x++) {
            for (let y = 0; y < grid.height; y++) {
                rects.push(
                    <rect
                        key={`un-${x}-${y}`}
                        x={x * mpc}
                        y={y * mpc}
                        width={mpc}
                        height={mpc}
                        fill="rgba(255, 255, 255, 0.012)"
                        stroke="rgba(255, 255, 255, 0.025)"
                        strokeWidth={0.1}
                    />
                );
            }
        }
        return rects;
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
                fill="rgba(0, 242, 254, 0.42)"
                stroke="rgba(0, 242, 254, 0.75)"
                strokeWidth={0.25}
            />
        ));
    }, [worldData, exploredCells]);

    // Filter tree visuals strictly to the active search area with slight padding
    const activeTreeVisuals = useMemo(() => {
        if (!worldData || !worldData.tree_visuals) return [];
        const { world, tree_visuals } = worldData;
        const ox = world.origin_x || 0;
        const oy = world.origin_y || 0;
        const wm = world.width_m;
        const hm = world.height_m;

        return tree_visuals.filter(t => {
            const tx = t.x - ox;
            const ty = t.y - oy;
            return tx >= -2.0 && tx <= wm + 2.0 && ty >= -2.0 && ty <= hm + 2.0;
        });
    }, [worldData]);

    if (!worldData) {
        return (
            <div className="panel flex-col items-center" style={{ flex: 1, justifyContent: 'center' }}>
                <Map size={48} className="text-muted" style={{ marginBottom: '16px' }} />
                <span className="text-muted">Waiting for world data...</span>
            </div>
        );
    }

    const { world, obstacles } = worldData;
    const viewBox = `0 0 ${world.width_m} ${world.height_m}`;
    const originX = world.origin_x || 0;
    const originY = world.origin_y || 0;

    const getDroneColor = (droneId: string): string => {
        const idx = parseInt(droneId.replace('drone_', ''), 10);
        return DRONE_PALETTE[idx % DRONE_PALETTE.length] || '#ffffff';
    };

    // Calculate heading angle in degrees from velocity or trajectory
    const getDroneHeading = (drone: DroneTelemetry): number => {
        if (drone.vx !== undefined && drone.vy !== undefined && (Math.abs(drone.vx) > 0.1 || Math.abs(drone.vy) > 0.1)) {
            return (Math.atan2(drone.vy, drone.vx) * 180) / Math.PI;
        }
        const pts = history[drone.id];
        if (pts && pts.length >= 2) {
            const last = pts[pts.length - 1];
            const prev = pts[pts.length - 2];
            const dx = last.x - prev.x;
            const dy = last.y - prev.y;
            if (Math.hypot(dx, dy) > 0.05) {
                return (Math.atan2(dy, dx) * 180) / Math.PI;
            }
        }
        return 0;
    };

    const isForestMap = worldData.id === 'forest_sar' || (worldData.grid ? worldData.grid.meters_per_cell >= 6.0 : false);
    const baseCampCenter = isForestMap ? { x: -150.0 - originX, y: 150.0 - originY } : null;

    const renderMapSvg = () => (
        <svg
            viewBox={viewBox}
            style={{
                width: '100%',
                height: '100%',
                maxHeight: '100%',
                maxWidth: '100%',
                borderRadius: '6px'
            }}
        >
            <defs>
                <clipPath id="sar-boundary-clip">
                    <rect x="0" y="0" width={world.width_m} height={world.height_m} rx="4" />
                </clipPath>
            </defs>

            {/* Background & Operational SAR Boundary */}
            <rect x="0" y="0" width={world.width_m} height={world.height_m} fill="#070a0e" stroke="rgba(0, 242, 254, 0.45)" strokeWidth="0.8" rx="4" />

            <g clipPath="url(#sar-boundary-clip)">
                {/* Search Grid Background */}
                <g className="unsearched-layer">{unsearchedRects}</g>
                <g className="grid-layer">{gridLines}</g>
                <g className="explored-layer">{exploredRects}</g>

                {/* Base Camp Staging Area & Drone Pads (if Forest SAR) */}
                {baseCampCenter && (
                    <g className="base-camp-layer">
                        {/* Staging perimeter */}
                        <circle
                            cx={baseCampCenter.x}
                            cy={baseCampCenter.y}
                            r={32.0}
                            fill="rgba(0, 242, 254, 0.03)"
                            stroke="rgba(0, 242, 254, 0.35)"
                            strokeWidth={0.6}
                            strokeDasharray="3,3"
                        />
                        <text
                            x={baseCampCenter.x}
                            y={baseCampCenter.y - 33.5}
                            fill="rgba(0, 242, 254, 0.85)"
                            fontSize={3.4}
                            textAnchor="middle"
                            fontFamily="var(--font-mono)"
                            fontWeight={700}
                            letterSpacing="0.5px"
                        >
                            BASE CAMP / STAGING HQ
                        </text>

                        {/* Drone Launch Pads */}
                        {worldData.drone_spawns && worldData.drone_spawns.map((s, idx) => {
                            const px = s.x - originX;
                            const py = s.y - originY;
                            return (
                                <g key={`pad-${idx}`} transform={`translate(${px}, ${py})`}>
                                    <rect x="-2.2" y="-2.2" width="4.4" height="4.4" fill="rgba(0, 242, 254, 0.12)" stroke="rgba(0, 242, 254, 0.5)" strokeWidth="0.3" rx="0.5" />
                                    <text y="1.0" fill="rgba(0, 242, 254, 0.9)" fontSize={2.2} textAnchor="middle" fontFamily="var(--font-mono)" fontWeight={700}>
                                        P{idx}
                                    </text>
                                </g>
                            );
                        })}
                    </g>
                )}

                {/* Search Sectors Layer */}
                {worldData.sectors && worldData.sectors.length > 0 && (
                    <g className="sectors-layer">
                        {worldData.sectors.map(s => {
                            const scx = s.center.x - originX;
                            const scy = s.center.y - originY;
                            return (
                                <g key={s.id}>
                                    <circle
                                        cx={scx}
                                        cy={scy}
                                        r={s.radius}
                                        fill="rgba(0, 242, 254, 0.02)"
                                        stroke="rgba(0, 242, 254, 0.3)"
                                        strokeWidth={0.6}
                                        strokeDasharray="4,4"
                                    />
                                    <text
                                        x={scx}
                                        y={scy - s.radius - 2.5}
                                        fill="rgba(0, 242, 254, 0.85)"
                                        fontSize={3.4}
                                        textAnchor="middle"
                                        fontFamily="var(--font-mono)"
                                        fontWeight={700}
                                        letterSpacing="0.5px"
                                    >
                                        {s.name}
                                    </text>
                                </g>
                            );
                        })}
                    </g>
                )}

                {/* Subtle Vegetation Canopy Texture (clipped to active SAR area) */}
                {activeTreeVisuals.length > 0 && (
                    <g className="trees-canopy-layer">
                        {activeTreeVisuals.map((t, idx) => {
                            const tx = t.x - originX;
                            const ty = t.y - originY;
                            const radius = 1.3 * (t.scale || 1.0);
                            return (
                                <g key={`canopy-${idx}`} opacity={0.45}>
                                    <circle
                                        cx={tx}
                                        cy={ty}
                                        r={radius}
                                        fill="rgba(34, 110, 50, 0.22)"
                                        stroke="rgba(46, 140, 70, 0.35)"
                                        strokeWidth={0.15}
                                    />
                                    <circle
                                        cx={tx}
                                        cy={ty}
                                        r={0.25}
                                        fill="rgba(90, 60, 30, 0.6)"
                                    />
                                </g>
                            );
                        })}
                    </g>
                )}

                {/* Collision-Relevant Obstacles Layer */}
                <g className="obstacles-layer">
                    {obstacles.map(obs => {
                        const ox1 = obs.aabb.min_x - originX;
                        const oy1 = obs.aabb.min_y - originY;
                        const w = obs.aabb.max_x - obs.aabb.min_x;
                        const h = obs.aabb.max_y - obs.aabb.min_y;
                        const cx = ox1 + w / 2;
                        const cy = oy1 + h / 2;

                        if (obs.type === 'tree') {
                            // Render collision tree trunks as distinct compact warning glyphs, not opaque squares
                            return (
                                <g key={obs.id} transform={`translate(${cx}, ${cy})`}>
                                    <circle r={0.9} fill="rgba(245, 158, 11, 0.4)" stroke="#f59e0b" strokeWidth={0.35} />
                                    <circle r={0.35} fill="#f59e0b" />
                                </g>
                            );
                        }

                        const isDebris = obs.type === 'debris' || obs.type === 'rubble';
                        const isBuilding = obs.type === 'building' || obs.type === 'houses';

                        return (
                            <g key={obs.id}>
                                <rect
                                    x={ox1}
                                    y={oy1}
                                    width={w}
                                    height={h}
                                    fill={isBuilding ? "rgba(59, 130, 246, 0.55)" : isDebris ? "rgba(217, 119, 6, 0.6)" : "rgba(75, 85, 99, 0.7)"}
                                    stroke={isBuilding ? "rgba(96, 165, 250, 0.9)" : isDebris ? "rgba(251, 191, 36, 0.9)" : "rgba(156, 163, 175, 0.9)"}
                                    strokeWidth={0.45}
                                    rx={1}
                                />
                                {isBuilding && w > 8 && (
                                    <text x={cx} y={cy + 1.0} fill="#ffffff" fontSize={2.2} textAnchor="middle" fontFamily="var(--font-mono)" fontWeight={700} opacity={0.8}>
                                        {obs.id.replace('base_ops_building', 'HQ').replace('house_', 'H')}
                                    </text>
                                )}
                            </g>
                        );
                    })}
                </g>

                {/* Ground Truth Victims Layer (Configured locations with live detection status) */}
                <g className="victims-gt-layer">
                    {victims.map(v => {
                        const cx = (v.world_x ?? 0) - originX;
                        const cy = (v.world_y ?? 0) - originY;
                        const isDet = Boolean(v.detected);
                        return (
                            <g key={`gt-${v.id}`} transform={`translate(${cx}, ${cy})`}>
                                {/* Radar pulse ring */}
                                <circle
                                    r={3.6}
                                    fill="none"
                                    stroke={isDet ? "rgba(0, 255, 136, 0.6)" : "rgba(255, 68, 68, 0.65)"}
                                    strokeWidth={0.4}
                                    strokeDasharray={isDet ? undefined : "1,1"}
                                />
                                <circle
                                    r={2.2}
                                    fill={isDet ? "#00ff87" : "#ff4444"}
                                    stroke="#ffffff"
                                    strokeWidth={0.4}
                                />
                                <text
                                    y={isDet ? 0.7 : -3.2}
                                    fill={isDet ? "#000" : "#ff4444"}
                                    fontSize={isDet ? 1.8 : 2.0}
                                    fontWeight={800}
                                    textAnchor="middle"
                                    fontFamily="var(--font-mono)"
                                >
                                    {isDet ? "✓" : v.id.toUpperCase().replace("VICTIM_", "V")}
                                </text>
                            </g>
                        );
                    })}
                </g>

                {/* Tracked Victims Layer (Perception estimates) */}
                <g className="victims-layer">
                    {trackedVictims && trackedVictims.map(v => {
                        const cx = (v.world_x ?? 0) - originX;
                        const cy = (v.world_y ?? 0) - originY;
                        return (
                            <g key={v.id} transform={`translate(${cx}, ${cy})`}>
                                <rect x={-2.6} y={-2.6} width={5.2} height={5.2} fill="none" stroke="rgba(0, 200, 255, 0.85)" strokeWidth={0.45} strokeDasharray="1,1" />
                                <text y={-3.4} fill="#00c8ff" fontSize={1.9} fontWeight={700} textAnchor="middle" fontFamily="var(--font-mono)">
                                    {v.id}
                                </text>
                                {v.confidence && (
                                    <text y={4.8} fill="#00c8ff" fontSize={1.5} fontWeight={700} textAnchor="middle" fontFamily="var(--font-mono)">
                                        {(v.confidence * 100).toFixed(0)}%
                                    </text>
                                )}
                            </g>
                        );
                    })}
                </g>

                {/* Trajectories Layer */}
                <g className="trajectories-layer">
                    {Object.entries(history).map(([droneId, pts]) => {
                        if (pts.length < 2) return null;
                        const color = getDroneColor(droneId);
                        const pathData = `M ${pts[0].x - originX} ${pts[0].y - originY} ` + pts.slice(1).map(p => `L ${p.x - originX} ${p.y - originY}`).join(' ');
                        return (
                            <path
                                key={`path-${droneId}`}
                                d={pathData}
                                fill="none"
                                stroke={color}
                                strokeWidth={0.7}
                                opacity={0.65}
                            />
                        );
                    })}
                </g>

                {/* Start Positions Layer */}
                <g className="start-positions-layer">
                    {Object.entries(history).map(([droneId, pts]) => {
                        if (pts.length === 0) return null;
                        const color = getDroneColor(droneId);
                        return (
                            <g key={`start-${droneId}`} transform={`translate(${pts[0].x - originX}, ${pts[0].y - originY})`}>
                                <circle r={1.2} fill="transparent" stroke={color} strokeWidth={0.3} strokeDasharray="0.5,0.5" />
                                <circle r={0.35} fill={color} />
                            </g>
                        );
                    })}
                </g>

                {/* Drones Layer (with live heading orientation wedge) */}
                <g className="drones-layer">
                    {drones.map(d => {
                        const color = getDroneColor(d.id);
                        const dx = d.x - originX;
                        const dy = d.y - originY;
                        const headingDeg = getDroneHeading(d);

                        return (
                            <g key={d.id} transform={`translate(${dx}, ${dy})`} style={{ transition: 'transform 0.3s ease-out' }}>
                                {/* Directional Heading Chevron/Wedge */}
                                <g transform={`rotate(${headingDeg})`}>
                                    <polygon
                                        points="3.8,0 -2.2,-2.0 -1.0,0 -2.2,2.0"
                                        fill={color}
                                        opacity={0.9}
                                    />
                                </g>

                                {/* Center Drone Marker */}
                                <circle r={4.5} fill={color} opacity={0.18} />
                                <circle r={1.9} fill={color} stroke="#ffffff" strokeWidth={0.5} />
                                <text
                                    y={-3.8}
                                    fill="#ffffff"
                                    fontSize={2.6}
                                    textAnchor="middle"
                                    fontFamily="var(--font-mono)"
                                    fontWeight={800}
                                    style={{ textShadow: '0px 1px 3px rgba(0,0,0,0.95)' }}
                                >
                                    {d.id.replace('drone_', 'D')}
                                </text>
                            </g>
                        );
                    })}
                </g>
            </g>
        </svg>
    );

    const renderLegend = () => (
        <div style={{
            position: 'absolute', bottom: '12px', left: '12px', zIndex: 10,
            background: 'rgba(11, 15, 22, 0.92)', padding: '8px 12px', borderRadius: '6px',
            border: '1px solid rgba(255,255,255,0.08)',
            display: 'flex', flexDirection: 'column', gap: '5px', fontSize: '0.65rem', letterSpacing: '0.5px'
        }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, background: 'rgba(255,255,255,0.015)', border: '1px solid rgba(255,255,255,0.03)' }} />
                <span>UNSEARCHED CELL</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, background: 'rgba(0,242,254,0.42)', border: '1px solid rgba(0,242,254,0.75)' }} />
                <span>SEARCHED COVERAGE</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, borderRadius: '50%', background: 'rgba(34, 110, 50, 0.35)', border: '1px solid rgba(46, 140, 70, 0.6)' }} />
                <span>FOREST CANOPY</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 8, height: 8, borderRadius: '50%', background: 'rgba(245, 158, 11, 0.5)', border: '1px solid #f59e0b' }} />
                <span>TREE HAZARD / COLLISION</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 10, height: 10, background: 'rgba(59, 130, 246, 0.55)', border: '1px solid rgba(96, 165, 250, 0.9)' }} />
                <span>HQ / BUILDING</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 9, height: 9, borderRadius: '50%', background: '#ff4444', border: '1px solid #fff' }} />
                <span>VICTIM (TARGET)</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: 9, height: 9, borderRadius: '50%', background: '#00ff87', border: '1px solid #fff' }} />
                <span>VICTIM CONFIRMED</span>
            </div>
            {drones.map(d => (
                <div key={d.id} style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                    <div style={{ width: 9, height: 9, borderRadius: '50%', background: getDroneColor(d.id) }} />
                    <span>{d.id.replace('drone_', 'DRONE ')}</span>
                </div>
            ))}
        </div>
    );

    return (
        <>
            <div className="panel flex-col" style={{ flex: 1, position: 'relative', overflow: 'hidden' }}>
                <div style={{ position: 'absolute', top: '12px', left: '12px', right: '12px', zIndex: 10, display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start' }}>
                    <div style={{ background: 'rgba(11, 15, 22, 0.88)', padding: '4px 10px', borderRadius: '4px', border: '1px solid rgba(255,255,255,0.08)', display: 'flex', alignItems: 'center', gap: '6px', fontSize: '0.75rem', letterSpacing: '1px', color: 'var(--text-muted)' }}>
                        <Map size={14} /> 200m × 200m OPERATIONAL SAR GRID
                    </div>
                    <button
                        onClick={() => setIsExpanded(true)}
                        style={{ background: 'rgba(0, 242, 254, 0.1)', border: '1px solid var(--accent-cyan)', padding: '4px 10px', borderRadius: '4px', color: 'var(--accent-cyan)', fontSize: '0.75rem', cursor: 'pointer', fontWeight: 'bold' }}
                    >
                        EXPAND MAP
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
                    background: 'rgba(0,0,0,0.95)', zIndex: 9999,
                    display: 'flex', flexDirection: 'column', padding: '2rem'
                }}>
                    <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
                        <div style={{ display: 'flex', alignItems: 'center', gap: '16px' }}>
                            <h2 style={{ margin: 0, color: 'var(--accent-cyan)', letterSpacing: '2px', display: 'flex', alignItems: 'center', gap: '8px' }}>
                                <Map size={24} /> OPERATIONAL SAR GRID HEATMAP
                            </h2>
                            <div style={{ background: 'rgba(0, 242, 254, 0.15)', border: '1px solid var(--accent-cyan)', color: 'var(--accent-cyan)', padding: '4px 12px', borderRadius: '4px', fontSize: '1rem', fontWeight: 'bold', fontFamily: 'var(--font-mono)' }}>
                                COVERAGE: {coverage.toFixed(1)}%
                            </div>
                        </div>
                        <button
                            onClick={() => setIsExpanded(false)}
                            style={{ background: 'transparent', border: '1px solid var(--text-muted)', padding: '6px 16px', borderRadius: '4px', color: 'var(--text-main)', cursor: 'pointer', fontSize: '0.9rem' }}
                        >
                            CLOSE X
                        </button>
                    </div>

                    <div style={{ flex: 1, position: 'relative', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px', background: '#070a0e', overflow: 'hidden' }}>
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
