#ifndef SEQUENCEICUB_H
#define SEQUENCEICUB_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/Trajectory/wOcraTrajectory.h"

class sequence_iCub_01_Standing: public wocra::wOcraTaskSequenceBase
{
    public:
        sequence_iCub_01_Standing();
        virtual ~sequence_iCub_01_Standing();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
/*
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
*/
};


class sequence_iCub_02_VariableWeightHandTasks: public wocra::wOcraTaskSequenceBase
{
    public:
        sequence_iCub_02_VariableWeightHandTasks();
        virtual ~sequence_iCub_02_VariableWeightHandTasks();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
        wocra::wOcraTrajectory*                        leftHandTrajectory;
        int lHandIndex;

    private:
        wocra::wOcraVariableWeightsTaskManager*           tmLeftHand;
/*
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
*/
};



#endif // TASKSETROMEOBALANCE_H
