#include "sitting-demo/SittingDemoClient.h"
SittingDemoClient::SittingDemoClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    xDisp = 0.0;
    yDisp = 0.0;
    zDisp = -0.04;
}

SittingDemoClient::~SittingDemoClient()
{
    // add your code here...
}

bool SittingDemoClient::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("x")) {
        xDisp = rf.find("x").asDouble();
    }
    if (rf.check("y")) {
        yDisp = rf.find("y").asDouble();
    }
    if (rf.check("z")) {
        zDisp = rf.find("z").asDouble();
    }

    std::cout << "xDisp = " << xDisp << std::endl;
    std::cout << "yDisp = " << yDisp << std::endl;
    std::cout << "zDisp = " << zDisp << std::endl;
}

bool SittingDemoClient::initialize()
{
    std::cout << "\n\nInitializing...\n" << std::endl;
    yarp::os::Time::delay(3.0);

    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;

    Eigen::Vector3d waypoints = model->getCoMPosition();

    std::cout << "current CoM position: " << waypoints.transpose() << std::endl;
    waypoints(0) += xDisp;
    waypoints(1) += yDisp;
    waypoints(2) += zDisp;
    std::cout << "desired CoM position: " << waypoints.transpose() << std::endl;

    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::REVERSE_STOP;

    rootTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", waypoints, trajType, termStrategy);

    // rootTrajThread->setDisplacement(0.2);
    rootTrajThread->setGoalErrorThreshold(0.03);
    rootTrajThread->setMaxVelocity(0.01);
    rootTrajThread->start();

    std::cout << "\n\nInitialization complete. Begining movement." << std::endl;
    return true;
}

void SittingDemoClient::release()
{
    // add your code here...
}

void SittingDemoClient::loop()
{
    if(!rootTrajThread->isRunning()) {
        std::cout << "Movement finished. Stopping client." << std::endl;
        stop();
    }
}
