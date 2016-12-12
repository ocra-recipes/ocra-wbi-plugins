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

    if (rf.check("maxVel")) {
        maxVel = rf.find("maxVel").asDouble();
    } else {
        if (useMinJerk) {
            maxVel = 0.06;
        } else {
            maxVel = 0.1;
        }
    }

    if (rf.check("maxAcc")) {
        maxAcc = rf.find("maxAcc").asDouble();
    } else {
        maxAcc = maxVel;
    }

    if (rf.check("delay")) {
        contactReleaseDelay = rf.find("delay").asDouble();
    } else {
        contactReleaseDelay = 2.0;
    }

    return true;
}

void StandingDemoClient::printHelp()
{
    std::cout << "=======================================" << std::endl;
    std::cout << "------- StandingDemoClient HELP -------" << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "Valid args" << std::endl;
    std::cout << "--help --> Shows this message :)." << std::endl;
    std::cout << "--minJerk --> Uses a MinimumJerkTrajectory instead of a TimeOptimalTrajectory." << std::endl;
    std::cout << "--maxVel [double value] --> Sets the maximum velocity of the movement." << std::endl;
    std::cout << "--maxAcc [double value] --> Sets the maximum acceleration of the movement." << std::endl;
    std::cout << "--delay [double value] --> Sets the time to wait before deactivating the leg contacts (seconds)." << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "-------       END OF HELP       -------" << std::endl;
    std::cout << "=======================================" << std::endl;
}

bool StandingDemoClient::initialize()
{
    std::cout << "\n\n\n\n" << std::endl;
    std::cout << "====================================================================" << std::endl;
    std::cout << "Creating the following trajectory:" << std::endl;
    if (useMinJerk) {
        std::cout << "type: MinimumJerkTrajectory" << std::endl;
    } else {
        std::cout << "type: TimeOptimalTrajectory" << std::endl;
    }
    std::cout << "maxVel: " << maxVel << std::endl;
    std::cout << "maxAcc: " << maxAcc << std::endl;
    std::cout << "====================================================================" << std::endl;

    leftLegContactTask = std::make_shared<ocra_recipes::TaskConnection>("LeftUpperLegContact");
    rightLegContactTask = std::make_shared<ocra_recipes::TaskConnection>("RightUpperLegContact");

    Eigen::Vector3d comStartingPos = model->getCoMPosition();
    double zDisp = 0.15;
    if (useMinJerk){
        zDisp = 0.145;
    }

    /* Manual solution */
    Eigen::MatrixXd com_waypoints(3,2);
    com_waypoints << comStartingPos, comStartingPos;
    com_waypoints(0,0) = 0.0; // First move x forward to between the feet
    com_waypoints(0,1) = 0.0; // First move x forward to between the feet
    com_waypoints(2,1) += zDisp; // and move z upward

    std::cout << "com_waypoints\n" << com_waypoints << std::endl;

    std::list<Eigen::VectorXd> com_waypointList;
    for(int i=0; i<com_waypoints.cols(); ++i) {
        com_waypointList.push_back(com_waypoints.col(i));
    }

    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::STOP_THREAD;

    if (useMinJerk) {
        ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;
        comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", com_waypoints, trajType, termStrategy);
        comTrajThread->setMaxVelocity(maxVel);
    } else {
        ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::TIME_OPTIMAL;
        comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "ComTask", com_waypointList, trajType, termStrategy);
        comTrajThread->setMaxVelocityAndAcceleration(maxVel, maxAcc);
    }

    comTrajThread->start();


    // rootTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "RootCartesian", root_waypoints, trajType, termStrategy);
    // rootTrajThread->start();



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
