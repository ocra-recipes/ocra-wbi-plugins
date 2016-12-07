#include "standing-demo/StandingDemoClient.h"
StandingDemoClient::StandingDemoClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    // add your code here...
}

StandingDemoClient::~StandingDemoClient()
{
    // add your code here...
}

bool StandingDemoClient::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("minJerk")) {
        useMinJerk = true;
    } else {
        useMinJerk = false;
    }
    return true;
}

bool StandingDemoClient::initialize()
{
    // comTask = std::make_shared<ocra_recipes::TaskConnection>("ComTask");
    // rootTask = std::make_shared<ocra_recipes::TaskConnection>("RootCartesian");
    leftLegContactTask = std::make_shared<ocra_recipes::TaskConnection>("LeftUpperLegContact");
    rightLegContactTask = std::make_shared<ocra_recipes::TaskConnection>("RightUpperLegContact");

    Eigen::Vector3d comStartingPos = model->getCoMPosition();
    // Eigen::Vector3d rootStartingPos = model->getSegmentPosition("root_link").getTranslation();

    // Eigen::Vector3d leftFootPos = model->getSegmentPosition("l_foot").getTranslation();
    // Eigen::Vector3d rightFootPos = model->getSegmentPosition("r_foot").getTranslation();

    // std::cout << "comStartingPos\n" << comStartingPos.transpose() << std::endl;
    // std::cout << "rootStartingPos\n" << rootStartingPos.transpose() << std::endl;
    // std::cout << "leftFootPos\n" << leftFootPos.transpose() << std::endl;
    // std::cout << "rightFootPos\n" << rightFootPos.transpose() << std::endl;

    // Eigen::MatrixXd root_waypoints(3,1);
    // root_waypoints << rootStartingPos;
    // root_waypoints(0,0) = 0.0; // move x forward to between the feet
    // root_waypoints(2,0) += 0.2; // and move z upward 20cm
    // std::cout << "root_waypoints\n" << root_waypoints << std::endl;


    Eigen::MatrixXd com_waypoints(3,2);
    com_waypoints << comStartingPos, comStartingPos;
    com_waypoints(0,0) = 0.0; // First move x forward to between the feet
    com_waypoints(0,1) = 0.0; // First move x forward to between the feet
    com_waypoints(2,1) += 0.2; // and move z upward 20cm
    std::cout << "com_waypoints\n" << com_waypoints << std::endl;

    std::list<Eigen::VectorXd> com_waypointList;
    for(int i=0; i<3; ++i) {
        com_waypointList.push_back(com_waypoints.col(i));
    }
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::STOP_THREAD;

    if (useMinJerk) {
        ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;
        comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", com_waypoints, trajType, termStrategy);
        comTrajThread->setMaxVelocity(0.1);
    } else {
        ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::TIME_OPTIMAL;
        comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", com_waypointList, trajType, termStrategy);
    }

    comTrajThread->start();


    // rootTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RootCartesian", root_waypoints, trajType, termStrategy);
    // rootTrajThread->start();


    contactReleaseDelay = 2.0;
    contactsReleased = false;

    startTime = yarp::os::Time::now();


    return true;
}

void StandingDemoClient::release()
{
    // add your code here...
}

void StandingDemoClient::loop()
{
    if (!contactsReleased && ((yarp::os::Time::now() - startTime) >= contactReleaseDelay) ){
        deactivateLegContacts();
        contactsReleased = true;
    }

    if(!comTrajThread->isRunning()) {
        stop();
    }
}

void StandingDemoClient::deactivateLegContacts()
{
    std::cout << "Deactivating back leg contacts." << std::endl;
    leftLegContactTask->deactivate();
    rightLegContactTask->deactivate();
}
