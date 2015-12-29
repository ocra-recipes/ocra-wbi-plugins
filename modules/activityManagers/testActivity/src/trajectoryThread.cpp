#include <testActivity/trajectoryThread.h>

trajectoryThread::trajectoryThread(int period, const std::string& taskPortName, const std::string& trajectoryType):
controlThreadBase(period, taskPortName),
trajType(trajectoryType)
{
    setThreadType("trajectoryThread");
}

bool trajectoryThread::ct_threadInit()
{
    trajectory = new wocra::wOcraMinimumJerkTrajectory();
    Eigen::VectorXd startStateVector = getCurrentState();

    Eigen::VectorXd goalVector = Eigen::VectorXd::Zero(startStateVector.size());
    goalVector.head(3) = startStateVector.head(3) + Eigen::VectorXd::Constant(3, -0.2);

    std::cout << "startStateVector: " << startStateVector.transpose() << std::endl;
    std::cout << "goalVector: " << goalVector.transpose() << std::endl;

    trajectory->setWaypoints(currentStateVector.head(3), goalVector.head(3));

    return true;
}

void trajectoryThread::ct_threadRelease()
{
    std::cout<< "trajectoryThread: Trajectory thread finished.\n";
}

void trajectoryThread::ct_run()
{
    Eigen::VectorXd desiredState = Eigen::VectorXd::Zero(9);
    desiredState << trajectory->getDesiredValues(yarp::os::Time::now());
    yarp::os::Bottle desStateBottle;

    for(int i=0; i<desiredState.size(); i++)
    {
        desStateBottle.addDouble(desiredState(i));
    }

    outputPort.write(desStateBottle);
}
