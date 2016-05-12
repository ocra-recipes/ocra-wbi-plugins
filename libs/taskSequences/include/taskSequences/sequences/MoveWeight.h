#ifndef MOVEWEIGHT_H
#define MOVEWEIGHT_H

#include "../sequenceTools.h"

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "ocra/control/Trajectory/GaussianProcessTrajectory.h"

#include <smlt/smltUtilities.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <math.h>



class MoveWeight: public ocra::TaskSequence
{
    public:
        MoveWeight();
        ~MoveWeight();
    protected:
        virtual void doInit(ocra::Controller& c, ocra::Model& m);
        virtual void doUpdate(double time, ocra::Model& model, void** args);
    private:

        //CoM task
        ocra::CoMTaskManager*                    comTask;
        Eigen::Vector3d                                initialCoMPosition;

        //Right hand task
        bool useVarianceModulation;

        // ocra::SegPoseTaskManager* rightHandTask;
        ocra::SegCartesianTaskManager* rightHandTask;


        ocra::GaussianProcessTrajectory* rightHandTrajectory;
        Eigen::Vector3d rHandPosStart, rHandPosEnd, currentOptWaypoint;
        Eigen::VectorXd optVariables;

        double maxVariance;
        double resetTimeRight;

        Eigen::MatrixXd desiredPosVelAcc_rightHand;
        Eigen::VectorXd rightHandGoalPosition;
        Eigen::Vector3d rightHandGoalPosition_transformed, rightHandStartPosition_transformed;
        Eigen::VectorXd desiredVariance_rightHand, desiredWeights_rightHand;
        Eigen::Array3d varianceThresh;

        Eigen::VectorXd rightHandStaticWeight;



        //YARP
        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> costPortOut;
        yarp::os::BufferedPort<yarp::os::Bottle> optVarsPortIn;
        yarp::os::BufferedPort<yarp::os::Bottle> optParamsPortOut;
        std::string optVarsPortOut_name, costPortOut_name, optVarsPortIn_name, optParamsPortOut_name;

        yarp::os::Port r_hand_port;
        yarp::os::Port r_hand_start_port;
        yarp::os::Port r_hand_target_port;
        yarp::os::Port r_hand_waypoint_port;

        yarp::os::RpcClient rpcClientPort;

        Eigen::Vector3d gazeboTranslation;
        void connectYarpPorts();
        void sendFramePositionsToGazebo();
        void bottleEigenVector(yarp::os::Bottle& bottle, const Eigen::VectorXd& vecToBottle, const bool encapsulate=false);



        // weights
        Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);



        // control
        bool waitForSolver;
        bool waitForHomePosition;
        bool optimumFound;
        bool sequenceFinished;
        bool initTrigger;
        bool newOptVarsReceived;
        bool dataSent_AwaitReply;

        int testNumber;

        bool initializeStabilization;
        double stabilizationTime;
        bool deactivatingHandTask;


        bool useGoalCost;
        bool useTrackingCost;
        bool useEnergyCost;



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

        bool replayOptimalTrajectory;


        double bOptCovarianceScalingFactor;



        bool openLogFiles(const std::string testLogFilePathPrefix);
        bool closeLogFiles();
        void sendOptimizationParameters();
        void initializeTrajectory(double time);
        void executeTrajectory(double relativeTime,  ocra::Model& model);
        bool sendTestDataToSolver();
        double postProcessInstantaneousCosts();
        bool parseNewOptVarsBottle();
        bool isBackInHomePosition(const ocra::Model& model);
        bool attainedGoal(const ocra::Model& model);
        void calculateInstantaneousCost(const double time, const ocra::Model& model);
        double calculateGoalCost(const double time, const ocra::Model& model);
        double calculateTrackingCost(const double time, const ocra::Model& model);
        double calculateEnergyCost(const double time, const ocra::Model& model);
        bool returnToStablePosture(const double time, const ocra::Model& model);

        void setInitialWaypoints();
        void initializeOptimization();
        void checkSolverStatus();

        bool optimizationInProgress;

        bool printedOnce;
        bool movingToWeight;
};




#endif
