#ifndef TASKOPTIMIZATION_H
#define TASKOPTIMIZATION_H



#include "../sequenceTools.h"

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "ocra/control/Trajectory/GaussianProcessTrajectory.h"

#include <smlt/smltUtilities.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>


class TaskOptimization : public ocra::TaskSequence
{
    public:
        TaskOptimization();
        ~TaskOptimization();
    protected:
        virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& model, void** args);
    private:

        void connectYarpPorts();
        void bottleEigenVector(yarp::os::Bottle& bottle, const Eigen::VectorXd& vecToBottle, const bool encapsulate=false);
        bool attainedGoal(ocra::Model& model, int segmentIndex);
        bool isBackInHomePosition(ocra::Model& model, int segmentIndex);

        Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);
        void sendFramePositionsToGazebo();
        void sendOptimizationParameters();


        void calculateInstantaneousCost(const double time, const ocra::Model& model, int segmentIndex);
        double calculateGoalCost(const double time, const ocra::Model& model, int segmentIndex);
        double calculateTrackingCost(const double time, const ocra::Model& model, int segmentIndex);
        double calculateEnergyCost(const double time, const ocra::Model& model, int segmentIndex);
        // void checkAndCreateDirectory(const std::string dirPath)
        // {
        //     if (!boost::filesystem::exists(dirPath)) {
        //         boost::filesystem::create_directories(dirPath);
        //     }
        // }


        int dofIndex;

        ocra::GaussianProcessTrajectory* rightHandTrajectory;

        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> costPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortIn;
        yarp::os::BufferedPort<yarp::os::Bottle> optParamsPortOut;


        yarp::os::Port r_hand_port, r_hand_start_port, r_hand_target_port, r_hand_waypoint_port, obstacle_port;


        std::string optVarsPortOut_name, costPortOut_name, optVarsPortIn_name, optParamsPortOut_name;

        Eigen::VectorXd optVariables;

        Eigen::Vector3d rHandPosStart, rHandPosEnd, currentOptWaypoint;

        int rHandIndex;

        ocra::VariableWeightsTaskManager* rightHandTask;

        double maxVariance;

        double resetTimeRight;

        double bOptCovarianceScalingFactor;

        Eigen::MatrixXd desiredPosVelAcc_rightHand;
        Eigen::VectorXd rightHandGoalPosition;
        Eigen::Vector3d rightHandGoalPosition_transformed, rightHandStartPosition_transformed, obstacle3DGazeboPosition;

        Eigen::VectorXd desiredVariance_rightHand, desiredWeights_rightHand;


        bool initTrigger;

        Eigen::Array3d varianceThresh;

        bool waitForSolver, dataSent_AwaitReply, newOptVarsReceived;

        bool useGoalCost, useTrackingCost, useEnergyCost;

        int waitCount;

        bool optimumFound, sequenceFinished, waitForHomePosition;


        int testNumber;
        bool logTrajectoryData;
        std::string rootLogFilePathPrefix;
        std::ofstream   totalInstantaneousCostFile,
                        goalInstantaneousCostFile,
                        trackingInstantaneousCostFile,
                        energyInstantaneousCostFile,

                        realTrajectoryFile;


        Eigen::MatrixXd goalCostMat, trackingCostMat, energyCostMat;
        int costIterCounter;

        double energyCostScalingFactor;


        bool openLogFiles(const std::string logFilePathPrefix="./");
        bool closeLogFiles();

        void initializeTrajectory(double time);
        void executeTrajectory(double relativeTime,  ocra::Model& model);
        bool parseNewOptVarsBottle();

        double obstacleTime;

        void insertObstacle();
        void removeObstacle();

        bool sendTestDataToSolver();

        double postProcessInstantaneousCosts();


        bool replayOptimalTrajectory;



        bool runObstacleTest_1D, runObstacleTest_3D, runArmCrossingTest, useVarianceModulation;

        void obstacleTest_UpdateThread(double time, ocra::Model& model);
        void ArmCrossingTest_UpdateThread(double time, ocra::Model& model);


};



#endif
