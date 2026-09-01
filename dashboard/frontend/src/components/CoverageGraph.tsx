import React from 'react';

interface CoverageGraphProps {
    history: { time: number; coverage: number }[];
}

export function CoverageGraph({ history }: CoverageGraphProps) {
    const [isExpanded, setIsExpanded] = React.useState(false);

    // If modal is active, listen for escape key
    React.useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.key === 'Escape') setIsExpanded(false);
        };
        if (isExpanded) window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [isExpanded]);

    if (history.length === 0) {
        return (
            <div className="panel" style={{ flex: 1, minHeight: '250px', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                <span className="text-muted">Waiting for mission start...</span>
            </div>
        );
    }

    const renderGraphSvg = (svgWidth: number, svgHeight: number) => {
        const padding = 20;
        const paddingLeft = 40;
        const paddingBottom = 30;

        const maxTime = Math.max(10, ...history.map(d => d.time)); // At least 10s width
        const minTime = 0;

        // Convert data to SVG coordinates
        const getX = (t: number) => paddingLeft + ((t - minTime) / maxTime) * (svgWidth - paddingLeft - padding);
        const getY = (c: number) => svgHeight - paddingBottom - (c / 100) * (svgHeight - paddingBottom - padding);

        // Build the SVG path
        const pathData = `M ${getX(history[0].time)} ${getY(history[0].coverage)} ` +
            history.slice(1).map(p => `L ${getX(p.time)} ${getY(p.coverage)}`).join(' ');

        // Area under the curve
        const areaData = pathData + ` L ${getX(history[history.length - 1].time)} ${getY(0)} L ${getX(history[0].time)} ${getY(0)} Z`;

        const latest = history[history.length - 1];

        return (
            <svg viewBox={`0 0 ${svgWidth} ${svgHeight}`} preserveAspectRatio="none" style={{ width: '100%', height: '100%' }}>
                {/* Axes */}
                <line x1={paddingLeft} y1={padding} x2={paddingLeft} y2={svgHeight - paddingBottom} stroke="rgba(255,255,255,0.2)" strokeWidth={1} />
                <line x1={paddingLeft} y1={svgHeight - paddingBottom} x2={svgWidth - padding} y2={svgHeight - paddingBottom} stroke="rgba(255,255,255,0.2)" strokeWidth={1} />

                {/* Y-axis Labels */}
                {[0, 25, 50, 75, 100].map(val => (
                    <g key={`y-${val}`}>
                        <line x1={paddingLeft} y1={getY(val)} x2={svgWidth - padding} y2={getY(val)} stroke="rgba(255,255,255,0.05)" strokeWidth={1} />
                        <text x={paddingLeft - 8} y={getY(val) + 4} fill="var(--text-muted)" fontSize={10} textAnchor="end" fontFamily="var(--font-mono)">{val}%</text>
                    </g>
                ))}

                {/* X-axis Labels */}
                {[0, maxTime/4, maxTime/2, maxTime*0.75, maxTime].map((val, i) => (
                    <g key={`x-${i}`}>
                        <line x1={getX(val)} y1={svgHeight - paddingBottom} x2={getX(val)} y2={svgHeight - paddingBottom + 5} stroke="rgba(255,255,255,0.2)" strokeWidth={1} />
                        <text x={getX(val)} y={svgHeight - paddingBottom + 16} fill="var(--text-muted)" fontSize={10} textAnchor="middle" fontFamily="var(--font-mono)">{Math.round(val)}s</text>
                    </g>
                ))}

                {/* Area */}
                <path d={areaData} fill="rgba(0, 242, 254, 0.1)" />

                {/* Line */}
                <path d={pathData} fill="none" stroke="var(--accent-cyan)" strokeWidth={2} />

                {/* Current Data Point */}
                <circle cx={getX(latest.time)} cy={getY(latest.coverage)} r={4} fill="var(--accent-cyan)" />
                <text x={getX(latest.time)} y={getY(latest.coverage) - 10} fill="var(--accent-cyan)" fontSize={12} fontWeight="bold" textAnchor="middle">
                    {latest.coverage.toFixed(1)}%
                </text>
            </svg>
        );
    };

    const latestCov = history[history.length - 1].coverage;

    return (
        <>
            <div className="panel flex-col" style={{ flex: 1, minHeight: '250px' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                    <div className="panel-title" style={{ margin: 0 }}>Coverage vs Time</div>
                    <button
                        onClick={() => setIsExpanded(true)}
                        style={{ background: 'rgba(0, 242, 254, 0.1)', border: '1px solid var(--accent-cyan)', padding: '4px 10px', borderRadius: '4px', color: 'var(--accent-cyan)', fontSize: '0.75rem', cursor: 'pointer', fontWeight: 'bold' }}
                    >
                        VIEW GRAPH
                    </button>
                </div>

                <div style={{ flex: 1, width: '100%', height: '100%', position: 'relative' }}>
                    {renderGraphSvg(400, 200)}
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
                            COVERAGE VS MISSION TIME
                        </h2>
                        <button
                            onClick={() => setIsExpanded(false)}
                            style={{ background: 'transparent', border: '1px solid var(--text-muted)', padding: '6px 16px', borderRadius: '4px', color: 'var(--text-main)', cursor: 'pointer', fontSize: '0.9rem' }}
                        >
                            CLOSE X
                        </button>
                    </div>

                    <div style={{ flex: 1, position: 'relative', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px', background: '#090a0f', overflow: 'hidden' }}>
                        <div style={{ width: '100%', height: '100%', padding: '2rem', boxSizing: 'border-box' }}>
                            {renderGraphSvg(1000, 500)}
                        </div>
                        <div style={{ position: 'absolute', bottom: '24px', right: '24px', fontSize: '1.2rem', fontWeight: 'bold', color: 'var(--accent-cyan)' }}>
                            Final coverage: {latestCov.toFixed(1)}%
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}
