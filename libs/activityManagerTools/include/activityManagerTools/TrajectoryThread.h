#ifndef TRAJECTORYTHREAD_H
#define TRAJECTORYTHREAD_H

#include <activityManagerTools/ControlThreadBase.h>
#include <wocra/Trajectory/wOcraTrajectories.h>
#include <yarp/os/Time.h>
#include <iostream>

enum TRAJECTORY_TYPE
{
    MIN_JERK,
    LIN_INTERP,
    GAUSSIAN_PROCESS
};

enum TERMINATION_STRATEGY
{
    BACK_AND_FORTH,
    STOP_THREAD,
    WAIT,
    STOP_THREAD_DEACTIVATE,
    WAIT_DEACTIVATE
};

class TrajectoryThread : public ControlThreadBase
{

public:
    TrajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE = MIN_JERK, const TERMINATION_STRATEGY _terminationStrategy = STOP_THREAD);

    virtual bool ct_threadInit();
    virtual void ct_threadRelease();
    virtual void ct_run();
    virtual std::string getThreadType(){return "TrajectoryThread";}


    // Setters
    bool setDisplacement(double dispDouble);
    bool setDisplacement(const Eigen::VectorXd& displacementVector);
    bool setTrajectoryWaypoints(const Eigen::MatrixXd& userWaypoints, bool containsStartingWaypoint=false);
    void setTerminationStrategy(const TERMINATION_STRATEGY newTermStrat){terminationStrategy = newTermStrat;}
    void setGoalErrorThreshold(const double newErrorThresh){errorThreshold = newErrorThresh;}
    void setUseVarianceModulation(bool newVarMod){useVarianceModulation = newVarMod;}

    void setMeanWaypoints(std::vector<bool>& isMeanWaypoint);
    void setVarianceWaypoints(std::vector<bool>& isVarWaypoint);
    void setOptimizationWaypoints(std::vector<bool>& isOptWaypoint);
    void setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptimize);


    // Getters
    Eigen::VectorXd getBayesianOptimizationVariables();

    // General assesment functions
    bool goalAttained();


protected:


    Eigen::VectorXd varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta = 1.0);
    // void getTaskWeightDimension();
    void flipWaypoints();

    Eigen::MatrixXd userWaypoints;
    TRAJECTORY_TYPE trajType;
    TERMINATION_STRATEGY terminationStrategy;

    wocra::wOcraTrajectory* trajectory;


    double maximumVariance;
    bool useVarianceModulation;
    Eigen::VectorXd desiredVariance;
    Eigen::ArrayXd varianceThresh;

    Eigen::VectorXd startStateVector;
    Eigen::VectorXd goalStateVector;
    Eigen::MatrixXd allWaypoints;
    double errorThreshold;

    Eigen::VectorXd desiredState;
    yarp::os::Bottle desStateBottle;

    bool printWaitingNoticeOnce;

    double deactivationDelay;
    double deactivationTimeout;
    bool deactivationLatch;

};
#endif
