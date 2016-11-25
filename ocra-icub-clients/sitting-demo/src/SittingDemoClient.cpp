#include "sitting-demo/SittingDemoClient.h"
SittingDemoClient::SittingDemoClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    // add your code here...
}

SittingDemoClient::~SittingDemoClient()
{
    // add your code here...
}

bool SittingDemoClient::initialize()
{
    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;

    Eigen::Vector3d waypoints(-0.12, -0.105, 0.22);

    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::REVERSE_STOP;

    rootTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RootCartesian", waypoints, trajType, termStrategy);

    // rootTrajThread->setDisplacement(0.2);
    rootTrajThread->setGoalErrorThreshold(0.03);
    rootTrajThread->setMaxVelocity(0.01);
    rootTrajThread->start();
    return true;
}

void SittingDemoClient::release()
{
    // add your code here...
}

void SittingDemoClient::loop()
{
    // add your code here...
}
