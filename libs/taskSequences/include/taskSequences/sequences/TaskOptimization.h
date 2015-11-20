#ifndef TASKOPTIMIZATION_H
#define TASKOPTIMIZATION_H

#include "../sequenceTools.h"

#include <wocra/Tasks/wOcraTaskSequenceBase.h>
#include <wocra/Trajectory/wOcraGaussianProcessTrajectory.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

class TaskOptimization : public wocra::wOcraTaskSequenceBase
{
    public:
        TaskOptimization();
        ~TaskOptimization();
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:

        void connectToSolverPorts();
        void bottleEigenVector(yarp::os::Bottle& bottle, const Eigen::VectorXd& vecToBottle, const bool encapsulate=false);

        wocra::wOcraSegCartesianTaskManager* tmRightHandCart;

        wocra::wOcraGaussianProcessTrajectory* leftHandTrajectory;
        wocra::wOcraGaussianProcessTrajectory* rightHandTrajectory;

        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> costPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortIn;

        yarp::os::Port l_hand_port, l_hand_target_port, r_hand_port, r_hand_target_port;


        std::string optVarsPortOut_name, costPortOut_name, optVarsPortIn_name;

        Eigen::VectorXd optVariables;

        Eigen::Vector3d rHandPosStart, rHandPosEnd;




        int lHandIndex, rHandIndex;

        wocra::wOcraVariableWeightsTaskManager* leftHandTask;

        wocra::wOcraVariableWeightsTaskManager* rightHandTask;

        double maxVariance;

        double resetTimeLeft, resetTimeRight;

        bool attainedGoal(wocra::wOcraModel& state, int segmentIndex);

        Eigen::MatrixXd desiredPosVelAcc_leftHand, desiredPosVelAcc_rightHand;
        Eigen::VectorXd currentDesiredPosition_leftHand;

        Eigen::VectorXd currentDesiredPosition_rightHand;

        Eigen::VectorXd desiredVariance_leftHand, desiredVariance_rightHand, desiredWeights_leftHand, desiredWeights_rightHand;

        Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);

        bool initTrigger;

        Eigen::Array3d varianceThresh;



};



#endif
