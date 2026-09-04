import React, { useState } from 'react';
import { ExperimentList } from './ExperimentList';
import { MissionDetails } from './MissionDetails';
import { FinalEvaluationCard } from './FinalEvaluationCard';

export function DatabaseViewer() {
    const [selectedMissionId, setSelectedMissionId] = useState<string | null>(null);

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '16px', flex: 1, minHeight: 0, padding: '16px' }}>
            {!selectedMissionId ? (
                <>
                    <div style={{ display: 'flex', gap: '16px', height: '160px' }}>
                        <FinalEvaluationCard />
                        <div className="panel flex-col" style={{ flex: 1, justifyContent: 'center', alignItems: 'center' }}>
                            <div className="panel-title" style={{ marginBottom: '8px' }}>SCALABILITY NOTES</div>
                            <div className="text-muted" style={{ fontSize: '0.85rem', textAlign: 'center', maxWidth: '80%' }}>
                                System integration successfully demonstrated up to 6 drones.<br/>
                                N=2 QMIX evaluation complete.<br/>
                                Detailed analysis available in final statistical report.
                            </div>
                        </div>
                    </div>
                    <div style={{ flex: 1, display: 'flex', minHeight: 0 }}>
                        <ExperimentList onSelectMission={setSelectedMissionId} />
                    </div>
                </>
            ) : (
                <MissionDetails missionId={selectedMissionId} onBack={() => setSelectedMissionId(null)} />
            )}
        </div>
    );
}
