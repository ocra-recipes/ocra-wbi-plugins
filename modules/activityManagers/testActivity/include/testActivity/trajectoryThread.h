#ifndef TRAJECTORYTHREAD_H
#define TRAJECTORYTHREAD_H

#include <testActivity/controlThreadBase.h>
#include <wocra/Trajectory/wOcraTrajectories.h>
#include <yarp/os/Time.h>
#include <iostream>

enum TRAJECTORY_TYPE
{
    MIN_JERK,
    LIN_INTERP,
    GAUSSIAN_PROCESS
};


class trajectoryThread : public controlThreadBase
{

public:
    trajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE = MIN_JERK, bool _stopAtGoal=true, bool _backAndForth=false);

    virtual bool ct_threadInit();
    virtual void ct_threadRelease();
    virtual void ct_run();

    virtual std::string getThreadType(){return "trajectoryThread";}


protected:
    Eigen::MatrixXd userWaypoints;
    TRAJECTORY_TYPE trajType;
    wocra::wOcraTrajectory* trajectory;

    Eigen::VectorXd desiredState, desiredVariance;
    double maximumVariance;
    Eigen::ArrayXd varianceThresh;
    yarp::os::Bottle desStateBottle;
    Eigen::VectorXd varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta = 1.0);

    void getTaskWeightDimension();
    int weightDimension;


    Eigen::VectorXd startStateVector, goalStateVector;

    bool stopAtGoal;

    bool goalAttained(const double errorThreshold = 0.03);

    void flipWaypoints();

    Eigen::MatrixXd allWaypoints;
    bool backAndForth;


};
#endif
