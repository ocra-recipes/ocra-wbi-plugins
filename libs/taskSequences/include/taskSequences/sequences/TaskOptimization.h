#ifndef TASKOPTIMIZATION_H
#define TASKOPTIMIZATION_H

#include "../sequenceTools.h"

#include <wocra/Tasks/wOcraTaskSequenceBase.h>
#include <wocra/Trajectory/wOcraGaussianProcessTrajectory.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>


class TaskOptimization : public wocra::wOcraTaskSequenceBase
{
    public:
        TaskOptimization();
        ~TaskOptimization();
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:

        void connectYarpPorts();
        void bottleEigenVector(yarp::os::Bottle& bottle, const Eigen::VectorXd& vecToBottle, const bool encapsulate=false);
        bool attainedGoal(wocra::wOcraModel& state, int segmentIndex);
        bool isBackInHomePosition(wocra::wOcraModel& state, int segmentIndex);

        Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);
        void sendFramePositionsToGazebo();
        void sendOptimizationParameters();


        double calculateInstantaneousCost(const double time, const wocra::wOcraModel& state, int segmentIndex);
        double calculateGoalCost(const double time, const wocra::wOcraModel& state, int segmentIndex);
        double calculateTrackingCost(const double time, const wocra::wOcraModel& state, int segmentIndex);
        double calculateEnergyCost(const double time, const wocra::wOcraModel& state, int segmentIndex);
        void checkAndCreateDirectory(const std::string dirPath)
        {
            if (!boost::filesystem::exists(dirPath)) {
                boost::filesystem::create_directories(dirPath);
            }
        }


        int dofIndex;

        wocra::wOcraGaussianProcessTrajectory* rightHandTrajectory;

        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> costPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortIn;
        yarp::os::BufferedPort<yarp::os::Bottle> optParamsPortOut;


        yarp::os::Port r_hand_port, r_hand_target_port, r_hand_waypoint_port;


        std::string optVarsPortOut_name, costPortOut_name, optVarsPortIn_name, optParamsPortOut_name;

        Eigen::VectorXd optVariables;

        Eigen::Vector3d rHandPosStart, rHandPosEnd, currentOptWaypoint;

        int rHandIndex;

        wocra::wOcraVariableWeightsTaskManager* rightHandTask;

        double maxVariance;

        double resetTimeRight;


        Eigen::MatrixXd desiredPosVelAcc_rightHand;
        Eigen::VectorXd rightHandGoalPosition;
        Eigen::Vector3d rightHandGoalPosition_transformed;

        Eigen::VectorXd desiredVariance_rightHand, desiredWeights_rightHand;


        bool initTrigger;

        Eigen::Array3d varianceThresh;

        bool waitForSolver, dataSent_AwaitReply, newOptVarsReceived;

        double totalCost;
        bool useGoalCost, useTrackingCost, useEnergyCost;

        int waitCount;

        bool optimumFound, waitForHomePosition;


        int testNumber;
        bool logTrajectoryData;
        std::string rootLogFilePathPrefix;
        std::ofstream   totalInstantaneousCostFile,
                        goalInstantaneousCostFile,
                        trackingInstantaneousCostFile,
                        energyInstantaneousCostFile;

        bool openLogFiles(const std::string logFilePathPrefix="./");
        bool closeLogFiles();


};



#endif
