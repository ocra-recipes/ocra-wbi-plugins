#ifndef TASKOPTIMIZATION_H
#define TASKOPTIMIZATION_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"
#include "wocra/Trajectory/wOcraGaussianProcessTrajectory.h"


class TaskOptimization : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:

        wocra::wOcraGaussianProcessTrajectory* leftHandTrajectory;
        wocra::wOcraGaussianProcessTrajectory* rightHandTrajectory;

};



#endif
