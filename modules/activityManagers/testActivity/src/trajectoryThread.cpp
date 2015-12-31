#include <testActivity/trajectoryThread.h>

#ifndef VAR_THRESH
#define VAR_THRESH 0.99
#endif

trajectoryThread::trajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE trajectoryType, const TERMINATION_STRATEGY _terminationStrategy):
controlThreadBase(period, taskPortName),
userWaypoints(waypoints),
trajType(trajectoryType),
terminationStrategy(_terminationStrategy),
printWaitingNoticeOnce(true),
errorThreshold(0.03),
useVarianceModulation(true),
deactivationDelay(0.0),
deactivationTimeout(5.0),
deactivationLatch(false)
{
    setThreadType("trajectoryThread");

    switch (trajType)
    {
        case MIN_JERK:
            trajectory = new wocra::wOcraMinimumJerkTrajectory();
            break;
        case LIN_INTERP:
            trajectory = new wocra::wOcraLinearInterpolationTrajectory();
            break;
        case GAUSSIAN_PROCESS:
            trajectory = new wocra::wOcraGaussianProcessTrajectory();
            break;
    }
}

bool trajectoryThread::ct_threadInit()
{
    if (trajType==GAUSSIAN_PROCESS)
    {
        desiredVariance = Eigen::VectorXd::Ones(weightDimension);
        varianceThresh = Eigen::ArrayXd::Constant(weightDimension, VAR_THRESH);
    }

    return setTrajectoryWaypoints(userWaypoints);
}

void trajectoryThread::ct_threadRelease()
{
    delete trajectory;
    trajectory = NULL;
    std::cout<< "\ntrajectoryThread: Trajectory thread finished.\n";
}

void trajectoryThread::ct_run()
{
    if (goalAttained() || deactivationLatch)
    {
        switch (terminationStrategy)
        {
            case BACK_AND_FORTH:
                flipWaypoints();
                setTrajectoryWaypoints(allWaypoints, true);
                break;
            case STOP_THREAD:
                stop();
                break;
            case STOP_THREAD_DEACTIVATE:
                if(deactivateTask()){
                    stop();
                }else{
                    std::cout << "[WARNING] Trajectory id = "<< controlThreadBase::threadId << " for task: " << originalTaskParams.name << " has attained its goal state, but cannot be deactivated." << std::endl;
                    yarp::os::Time::delay(1.0); // try again in one second.
                    deactivationDelay += 1.0;
                    if(deactivationDelay >= deactivationTimeout){
                        std::cout << "[WARNING] Deactivation timeout." << std::endl;
                        stop();
                    }
                }
                break;
            case WAIT:
                if (printWaitingNoticeOnce) {
                    std::cout << "Trajectory id = "<< controlThreadBase::threadId << " for task: " << originalTaskParams.name << " has attained its goal state. Awaiting new commands." << std::endl;
                    printWaitingNoticeOnce = false;
                }
                break;
            case WAIT_DEACTIVATE:
                if (printWaitingNoticeOnce) {
                    if(deactivateTask()){
                        std::cout << "Trajectory id = "<< controlThreadBase::threadId << " for task: " << originalTaskParams.name << " has attained its goal state. Deactivating task and awaiting new commands." << std::endl;
                        printWaitingNoticeOnce = false;
                        deactivationLatch = true;
                    }else{
                        std::cout << "Trajectory id = "<< controlThreadBase::threadId << " for task: " << originalTaskParams.name << " has attained its goal state and is awaiting new commands. [WARNING] Could not deactivate the task." << std::endl;
                        yarp::os::Time::delay(1.0); // try again in one second.
                        deactivationDelay += 1.0;
                        if(deactivationDelay >= deactivationTimeout){
                            printWaitingNoticeOnce = false;
                            std::cout << "[WARNING] Deactivation timeout." << std::endl;
                        }
                    }
                }
                break;
        }
    }
    else{
        if (!currentTaskParams.isActive) {
            activateTask();
        }

        desStateBottle.clear();
        if (trajType==GAUSSIAN_PROCESS)
        {
            Eigen::MatrixXd desiredState_tmp;
            trajectory->getDesiredValues(yarp::os::Time::now(), desiredState_tmp, desiredVariance);
            desiredState << desiredState_tmp;


            for(int i=0; i<desiredState.size(); i++)
            {
                desStateBottle.addDouble(desiredState(i));
            }
            if(useVarianceModulation)
            {
                Eigen::VectorXd desiredWeights = varianceToWeights(desiredVariance);
                for(int i=0; i<desiredWeights.size(); i++)
                {
                    desStateBottle.addDouble(desiredWeights(i));
                }
            }
        }
        else
        {
            desiredState << trajectory->getDesiredValues(yarp::os::Time::now());
            for(int i=0; i<desiredState.size(); i++)
            {
                desStateBottle.addDouble(desiredState(i));
            }
        }


        outputPort.write(desStateBottle);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd trajectoryThread::varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta)
{
    desiredVariance /= maximumVariance;
    desiredVariance = desiredVariance.array().min(varianceThresh); //limit desiredVariance to 0.99 maximum
    Eigen::VectorXd desiredWeights = (Eigen::VectorXd::Ones(desiredVariance.rows()) - desiredVariance) / beta;
    return desiredWeights;
}

bool trajectoryThread::goalAttained()
{
    return (goalStateVector - getCurrentState().head(weightDimension)).norm() <= errorThreshold;
}

void trajectoryThread::flipWaypoints()
{
    int nCols = allWaypoints.cols();
    Eigen::MatrixXd tmp(allWaypoints.rows(), nCols);

    int c_tmp = nCols - 1;
    for(int c=0; c<nCols; c++)
    {
        tmp.col(c_tmp) = allWaypoints.col(c);
        c_tmp--;
    }
    allWaypoints = tmp;
    goalStateVector = allWaypoints.rightCols(1);
}

bool trajectoryThread::setTrajectoryWaypoints(const Eigen::MatrixXd& userWaypoints, bool containsStartingWaypoint)
{
    Eigen::MatrixXd _userWaypoints = userWaypoints; // Copy waypoints first in case we use allWaypoints as an arg.
    if (weightDimension==_userWaypoints.rows())
    {
        if(containsStartingWaypoint)
        {
            allWaypoints = _userWaypoints;
        }
        else // Add starting waypoint
        {
            allWaypoints = Eigen::MatrixXd(weightDimension, _userWaypoints.cols()+1);

            startStateVector = getCurrentState();
            desiredState = Eigen::VectorXd::Zero(startStateVector.size());


            allWaypoints.col(0) << currentStateVector.head(weightDimension);
            for(int i=0; i<_userWaypoints.cols(); i++)
            {
                allWaypoints.col(i+1) << _userWaypoints.col(i);
            }
        }

        goalStateVector = allWaypoints.rightCols(1);

        trajectory->setWaypoints(allWaypoints);

        if (trajType==GAUSSIAN_PROCESS)
        {
            maximumVariance = dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->getMaxVariance();
        }

        printWaitingNoticeOnce=true;
        deactivationLatch = false;

        return true;
    }
    else
    {
        std::cout << "[ERROR](trajectoryThread::setTrajectoryWaypoints): The dimension (# DOF) of the waypoints you provided, " << _userWaypoints.rows() << ", does not match the dimension of the task, " << weightDimension <<". Thread not starting." << std::endl;
        return false;
    }
}



void trajectoryThread::setMeanWaypoints(std::vector<bool>& isMeanWaypoint)
{
    dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->setMeanWaypoints(isMeanWaypoint);
}

void trajectoryThread::setVarianceWaypoints(std::vector<bool>& isVarWaypoint)
{
    dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->setVarianceWaypoints(isVarWaypoint);
}

void trajectoryThread::setOptimizationWaypoints(std::vector<bool>& isOptWaypoint)
{
    dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->setOptimizationWaypoints(isOptWaypoint);
}

void trajectoryThread::setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptimize)
{
    dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->setDofToOptimize(dofToOptimize);
}

Eigen::VectorXd trajectoryThread::getBayesianOptimizationVariables()
{
    return dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->getBoptVariables();
}


bool trajectoryThread::setDisplacement(double dispDouble)
{
    return setDisplacement(Eigen::VectorXd::Constant(weightDimension, dispDouble));
}

bool trajectoryThread::setDisplacement(const Eigen::VectorXd& displacementVector)
{
    if (weightDimension == displacementVector.rows())
    {
        startStateVector = getCurrentState();
        Eigen::MatrixXd tmpWaypoints = Eigen::MatrixXd::Zero(weightDimension, 2);
        tmpWaypoints.col(0) << startStateVector;
        tmpWaypoints.col(1) << startStateVector + displacementVector;
        setTrajectoryWaypoints(tmpWaypoints, true);
        return true;
    }
    else{
        return false;
    }
}
