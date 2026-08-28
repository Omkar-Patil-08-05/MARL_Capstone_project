import React from 'react';
import { useTelemetry } from './hooks/useTelemetry';
import { Header } from './components/Header';
import { MissionOverview } from './components/MissionOverview';
import { DroneMap } from './components/DroneMap';
import { DroneCard } from './components/DroneCard';
import { AlertsFeed } from './components/AlertsFeed';
import { MissionProgress } from './components/MissionProgress';
import { MapSelection } from './components/MapSelection';

function App() {
    const { telemetry, worldData, mapRegistry, backendStatus, isConnected, alerts, droneHistory, startMission, stopMission, resetMission } = useTelemetry();

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

            <div className="layout-main">
                <div className="layout-left">
                    <DroneMap 
                        worldData={worldData} 
                        drones={telemetry ? telemetry.drones : []} 
                        history={droneHistory}
                        exploredCells={telemetry ? telemetry.explored_cells : []}
                        victims={telemetry ? telemetry.victims : []}
                    />
                </div>
                
                <div className="layout-right">
                    {telemetry?.drones.map(drone => (
                        <DroneCard key={drone.id} drone={drone} />
                    ))}
                    <AlertsFeed alerts={alerts} />
                </div>
            </div>

                    <MissionProgress telemetry={telemetry ? telemetry.mission : null} />
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
