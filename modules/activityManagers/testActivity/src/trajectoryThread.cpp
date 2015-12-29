#include <testActivity/trajectoryThread.h>

#ifndef VAR_THRESH
#define VAR_THRESH 0.99
#endif

trajectoryThread::trajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE trajectoryType, bool _stopAtGoal, bool _backAndForth):
controlThreadBase(period, taskPortName),
userWaypoints(waypoints),
trajType(trajectoryType),
stopAtGoal(_stopAtGoal),
backAndForth(_backAndForth),
weightDimension(0)
{
    setThreadType("trajectoryThread");
}

bool trajectoryThread::ct_threadInit()
{
    getTaskWeightDimension();
    if (weightDimension==userWaypoints.rows())
    {
        switch (trajType) {

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

        allWaypoints = Eigen::MatrixXd(weightDimension, userWaypoints.cols()+1);

        startStateVector = getCurrentState();
        desiredState = Eigen::VectorXd::Zero(startStateVector.size());


        allWaypoints.col(0) << currentStateVector.head(weightDimension);
        for(int i=0; i<userWaypoints.cols(); i++)
        {
            allWaypoints.col(i+1) << userWaypoints.col(i);
        }

        goalStateVector = allWaypoints.rightCols(1);

        trajectory->setWaypoints(allWaypoints);

        if (trajType==GAUSSIAN_PROCESS)
        {
            maximumVariance = dynamic_cast<wocra::wOcraGaussianProcessTrajectory*>(trajectory)->getMaxVariance();
        }
        return true;
    }
    else
    {
        std::cout << "[ERROR](): The dimension (# DOF) of the waypoints you provided, " << userWaypoints.rows() << ", does not match the dimension of the task, " << weightDimension <<". Thread not starting." << std::endl;
        return false;
    }
}

void trajectoryThread::ct_threadRelease()
{
    delete trajectory;
    trajectory = NULL;
    std::cout<< "trajectoryThread: Trajectory thread finished.\n";
}

void trajectoryThread::ct_run()
{
    if (goalAttained(0.045) && stopAtGoal) {
        if (backAndForth) {
            flipWaypoints();
            trajectory->setWaypoints(allWaypoints);
        }else{
            stop();
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

bool trajectoryThread::goalAttained(const double errorThreshold)
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
