import React, { useState } from 'react';
import { useTelemetry } from './hooks/useTelemetry';
import { Header } from './components/Header';
import { MissionOverview } from './components/MissionOverview';
import { DroneMap } from './components/DroneMap';
import { DroneCard } from './components/DroneCard';
import { AlertsFeed } from './components/AlertsFeed';
import { MissionProgress } from './components/MissionProgress';
import { MapSelection } from './components/MapSelection';
import { VictimDetectionPanel } from './components/VictimDetectionPanel';
import { CoverageGraph } from './components/CoverageGraph';
import { SwarmCoordinationStats } from './components/SwarmCoordinationStats';
import { MissionResults } from './components/MissionResults';
import { MissionSummary } from './components/MissionSummary';

function App() {
    const { telemetry, worldData, mapRegistry, backendStatus, isConnected, alerts, droneHistory, coverageHistory, startMission, stopMission, resetMission } = useTelemetry();
    const [activeView, setActiveView] = useState<'MAP' | 'CAMERA'>('MAP');
    const [showSummary, setShowSummary] = useState(true);

    const isMissionActive = backendStatus.state !== 'IDLE' && backendStatus.state !== 'ERROR';

    return (
        <div className="dashboard-container">
            <Header
                isConnected={isConnected}
                telemetry={telemetry ? telemetry.mission : null}
                backendStatus={backendStatus}
                onStop={stopMission}
                onReset={resetMission}
            />

            {!isMissionActive && mapRegistry && (
                <MapSelection
                    registry={mapRegistry}
                    backendStatus={backendStatus}
                    onStart={startMission}
                />
            )}

            {isMissionActive && (
                <>
                    <MissionOverview
                        telemetry={telemetry ? telemetry.mission : null}
                        victims={telemetry ? telemetry.victims : []}
                    />

            <div style={{
                display: 'grid',
                gridTemplateColumns: '300px 1fr 350px',
                gridTemplateRows: '55vh minmax(280px, auto) auto',
                gap: '16px',
                flex: 1,
                minHeight: 0
            }}>
                {/* Top Left: Drone Stats */}
                <div style={{ gridColumn: '1', gridRow: '1', display: 'flex', flexDirection: 'column', gap: '16px', overflowY: 'auto' }}>
                    {(telemetry?.drones || []).map(drone => (
                        <DroneCard
                            key={drone.id}
                            drone={drone}
                            activeView={activeView}
                            onToggleCamera={() => {}}
                        />
                    ))}
                </div>

                {/* Top Middle: SAR Heatmap */}
                <div style={{ gridColumn: '2', gridRow: '1', display: 'flex' }}>
                    <DroneMap
                        worldData={worldData}
                        drones={telemetry?.drones || []}
                        history={droneHistory}
                        exploredCells={telemetry?.explored_cells || []}
                        victims={telemetry?.victims || []}
                        trackedVictims={telemetry?.tracked_victims || []}
                        activeFrontiers={telemetry?.coordination?.active_frontiers}
                    />
                </div>

                {/* Top Right: Victims */}
                <div style={{ gridColumn: '3', gridRow: '1', display: 'flex', flexDirection: 'column', gap: '16px', overflowY: 'auto' }}>
                    <VictimDetectionPanel victims={telemetry ? (telemetry.tracked_victims || []) : []} />
                </div>

                {/* Middle Left+Center: Coverage Graph */}
                <div style={{ gridColumn: '1 / 3', gridRow: '2', display: 'flex' }}>
                    <CoverageGraph history={coverageHistory} />
                </div>

                {/* Middle Right: Coordination & Alerts */}
                <div style={{ gridColumn: '3', gridRow: '2', display: 'flex', flexDirection: 'column', gap: '16px', overflowY: 'auto' }}>
                    <SwarmCoordinationStats telemetry={telemetry} />
                    <AlertsFeed alerts={alerts} />
                </div>

                {/* Bottom: Results */}
                <div style={{ gridColumn: '1 / -1', gridRow: '3', display: 'flex' }}>
                    <MissionResults />
                </div>
            </div>

            {showSummary && (
                <MissionSummary
                    telemetry={telemetry}
                    status={backendStatus}
                    onClose={() => setShowSummary(false)}
                />
            )}
        </>
            )}

            {backendStatus.error && (
                <div style={{ color: 'var(--text-warning)', padding: '1rem', textAlign: 'center', backgroundColor: '#331111' }}>
                    <strong>SYSTEM ERROR:</strong> {backendStatus.error}
                </div>
            )}
        </div>
    );
}

export default App;
