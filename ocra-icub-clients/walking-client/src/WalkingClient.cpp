#include "walking-client/WalkingClient.h"
WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod),
_zmpParams(std::make_shared<ZmpControllerParams>(1, model->getMass(), model->getCoMPosition().operator()(2), 9.8) ),
_zmpController(std::make_shared<ZmpController>(loopPeriod, modelPtr, _zmpParams))
{

}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

bool WalkingClient::initialize()
{
    // Connect to feet wrench ports
    bool ok = portWrenchLeftFoot.open("/walkingClient/left_foot/wrench:i");
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/left_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        if (!yarp::os::Network::connect("/icubGazeboSim/left_foot/analog:o", portWrenchLeftFoot.getName().c_str())) {
            OCRA_ERROR("Impossible to connect to /icubGazeboSim/left_foot/analog:o");
            return false;
        }
    }
    ok = portWrenchRightFoot.open("/walkingClient/right_foot/wrench:i");
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/right_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        if (!yarp::os::Network::connect("/icubGazeboSim/right_foot/analog:o", portWrenchRightFoot.getName().c_str()) ) {
            OCRA_ERROR("Impossible to connect to /icubGazeboSim/right_foot/analog:o");
            return false;
        }
    }
    
    // Trajectory objects
    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::WAIT;
    int trajThreadPeriod = 10;
    _com_TrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(trajThreadPeriod, "ComTask", trajType, termStrategy);
    _com_TrajThread->setMaxVelocity(0.01);
    _com_TrajThread->setGoalErrorThreshold(0.01);
    if (!_com_TrajThread->start()) return false;
    
    return true;
}

void WalkingClient::release()
{
    // add your code here...
}

void WalkingClient::loop()
{
    // Read measurements
    Eigen::VectorXd rawLeftFootWrench(6);
    Eigen::VectorXd rawRightFootWrench(6);
    readFootWrench(LEFT_FOOT, rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, rawRightFootWrench);
    
    // Compute ZMP
    Eigen::Vector2d globalZMP; globalZMP.setZero();
    _zmpController->computeGlobalZMPFromSensors(rawLeftFootWrench, rawRightFootWrench, globalZMP);
    std::cout << "Global ZMP: " << globalZMP << std::endl;
    std::cout << "CoM - ZMP: " << this->model->getCoMPosition().topRows(2) - globalZMP << std::endl;
    // Compute ZMPPreviewController optimal input
    
}

bool WalkingClient::readFootWrench(FOOT whichFoot, Eigen::VectorXd &rawWrench) {
    yarp::sig::Vector * yRawFootWrench;
    switch (whichFoot) {
        case LEFT_FOOT:
            yRawFootWrench = portWrenchLeftFoot.read();
            break;
        case RIGHT_FOOT:
            yRawFootWrench = portWrenchRightFoot.read();
            break;
        default:
            break;
    }
    
    if (yRawFootWrench == NULL)
        return false;
    
    rawWrench = Eigen::VectorXd::Map(yRawFootWrench->data(), 6);
    return true;
}
