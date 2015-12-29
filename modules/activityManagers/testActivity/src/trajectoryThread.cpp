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
weightDimension(0),
errorThreshold(0.03)
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
    getTaskWeightDimension();

    return setTrajectoryWaypoints(userWaypoints);
}

void trajectoryThread::ct_threadRelease()
{
    delete trajectory;
    trajectory = NULL;
    std::cout<< "trajectoryThread: Trajectory thread finished.\n";
}

void trajectoryThread::ct_run()
{
    if (goalAttained())
    {
        switch (terminationStrategy)
        {
            case BACK_AND_FORTH:
                flipWaypoints();
                trajectory->setWaypoints(allWaypoints);
                break;
            case STOP_THREAD:
                stop();
                break;
            case WAIT:
                if (printWaitingNoticeOnce) {
                    std::cout << "Trajectory id = "<< controlThreadBase::threadId <<" has attained its goal state. Awaiting new commands..." << std::endl;
                    printWaitingNoticeOnce = false;
                }
                break;
        }
    }
    else{
        desStateBottle.clear();
        if (trajType==GAUSSIAN_PROCESS)

        {
            Eigen::MatrixXd desiredState_tmp;
            trajectory->getDesiredValues(yarp::os::Time::now(), desiredState_tmp, desiredVariance);
            desiredState << desiredState_tmp;

            Eigen::VectorXd desiredWeights = varianceToWeights(desiredVariance);

            for(int i=0; i<desiredState.size(); i++)
            {
                desStateBottle.addDouble(desiredState(i));
            }

            for(int i=0; i<desiredWeights.size(); i++)
            {
                desStateBottle.addDouble(desiredWeights(i));
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

void trajectoryThread::getTaskWeightDimension()
{
    yarp::os::Bottle message, reply;
    message.addString("getWeight");
    threadRpcClient.write(message, reply);
    if (reply.size() > 1)
    {
        std::cout << "weights = " << reply.toString() << std::endl;
        weightDimension = reply.size() - 1;
    }else{
        std::cout << "[ERROR](trajectoryThread::getTaskWeightDimension): Did not get a valid response from the task for its weight dimension. Setting to 0." << std::endl;
    }
    desiredVariance = Eigen::VectorXd::Ones(weightDimension);
    varianceThresh = Eigen::ArrayXd::Constant(weightDimension, VAR_THRESH);
}

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

bool trajectoryThread::setTrajectoryWaypoints(const Eigen::MatrixXd& _userWaypoints)
{
    if (weightDimension==_userWaypoints.rows())
    {
        allWaypoints = Eigen::MatrixXd(weightDimension, _userWaypoints.cols()+1);

        startStateVector = getCurrentState();
        desiredState = Eigen::VectorXd::Zero(startStateVector.size());


        allWaypoints.col(0) << currentStateVector.head(weightDimension);
        for(int i=0; i<_userWaypoints.cols(); i++)
        {
            allWaypoints.col(i+1) << _userWaypoints.col(i);
        }

        goalStateVector = allWaypoints.rightCols(1);

        trajectory->setWaypoints(allWaypoints);

        if (trajType==GAUSSIAN_PROCESS)
        {
            maximumVariance = dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->getMaxVariance();
        }

        printWaitingNoticeOnce=true;

        return true;
    }
    else
    {
        std::cout << "[ERROR](trajectoryThread::setTrajectoryWaypoints): The dimension (# DOF) of the waypoints you provided, " << _userWaypoints.rows() << ", does not match the dimension of the task, " << weightDimension <<". Thread not starting." << std::endl;
        return false;
    }
}
