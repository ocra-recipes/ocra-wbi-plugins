#ifndef TASKOPTIMIZATION_H
#define TASKOPTIMIZATION_H

#include "../sequenceTools.h"

#include <wocra/Tasks/wOcraTaskSequenceBase.h>
#include <wocra/Trajectory/wOcraGaussianProcessTrajectory.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

class TaskOptimization : public wocra::wOcraTaskSequenceBase
{
    public:
        ~TaskOptimization();
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:

        wocra::wOcraGaussianProcessTrajectory* leftHandTrajectory;
        wocra::wOcraGaussianProcessTrajectory* rightHandTrajectory;

        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> costPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortIn;
        // yarp::os::BufferedPort<yarp::os::Bottle> optVarsPort;


};



#endif
