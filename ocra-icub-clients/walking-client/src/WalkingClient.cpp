#include "walking-client/WalkingClient.h"

using namespace Eigen;

WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod),
_zmpParams(std::make_shared<ZmpControllerParams>(6e-4, model->getMass(), (model->getCoMPosition()).operator()(2), 9.8) ),
_zmpController(std::make_shared<ZmpController>(loopPeriod, modelPtr, _zmpParams)),
_isTestRun(false)
{

}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

bool WalkingClient::initialize()
{
    //TODO: Remember to remove the hardcorded /walkingClient name
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
    
    
    // CoM TaskConnection object. This is the object through which the CoM velocity will be set by this controller.
    std::string comTaskName("ComTask");
    _comTask = std::make_shared<ocra_recipes::TaskConnection>(comTaskName);
    _comTask->openControlPorts();
    
    
    // For testing and tuning purposes, generate ZMP trajectory that moves from one foot to the other.
    Eigen::Vector3d sep; sep.setZero();
    getFeetSeparation(sep);
    const double feetSeparation = sep(1);
    const int N = 2;
    const int period = this->getExpectedPeriod();
    const double tTrans = 3;
    _zmpTrajectory = generateZMPTrajectoryTEST(tTrans, feetSeparation, period, N);
    
    for (const Eigen::Vector2d &value: _zmpTrajectory)
        std::cout << value.transpose() << std::endl;
    
    
    // Is the controller undergoing a test run?
    _isTestRun = true;
    
    
    // If ZMP tests are being run, publish ZMP on port
    if (_isTestRun) {
        _zmpPort.open("/walkingClient/zmpError:o");
    }
    
    Eigen::Vector3d initialCOMPosition = this->model->getCoMPosition();
    _zmpParams->cz = initialCOMPosition(2);
    std::cout << "ZMPController object will run with the following parameters: " << std::endl;
    std::cout << "Gain k_f: " << _zmpParams->kf << std::endl;
    std::cout << "Total mass of the robot: " << _zmpParams->m << std::endl;
    std::cout << "Constant height of the COM: " << _zmpParams->cz << std::endl;
    std::cout << "Gravity acceleration: " << _zmpParams->g << std::endl;
    
    std::cout << "COM of the robot during init: " << this->model->getCoMPosition().transpose() << std::endl;
    
    return true;
}

void WalkingClient::release()
{
    
}

void WalkingClient::loop()
{
    static int el = 0;
    // Read measurements
    Eigen::VectorXd rawLeftFootWrench(6);
    Eigen::VectorXd rawRightFootWrench(6);
    readFootWrench(LEFT_FOOT, rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, rawRightFootWrench);
    
    
    // Compute ZMP
    Eigen::Vector2d globalZMP; globalZMP.setZero();
    _zmpController->computeGlobalZMPFromSensors(rawLeftFootWrench, rawRightFootWrench, globalZMP);
    
    
    // Testing
    Eigen::Vector2d dhd; dhd.setZero();
    //TODO: Write _zmpTrajectory to port
    if (_isTestRun) {
        _zmpController->computehd(globalZMP, _zmpTrajectory.rbegin()[el], dhd);
        Eigen::Vector2d error = globalZMP - _zmpTrajectory.rbegin()[el];
        publishZMPError(error);
        el++;
    }
    Eigen::Vector3d zeroAngVel = Eigen::Vector3d::Zero();
//    Eigen::Vector3d linVel( dhd(0), dhd(1), 0 );
    Eigen::Vector3d linVel( 0, 0.01, 0 );
    Eigen::Twistd refComVelocity( zeroAngVel, linVel );
    _desiredComState.setVelocity(refComVelocity);
    
    _comTask->setDesiredTaskStateDirect(_desiredComState);
    
    
    
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

std::vector<Eigen::Vector2d> WalkingClient::generateZMPTrajectoryTEST(const double tTrans,
                                                                      const double feetSeparation,
                                                                      const double timeStep,
                                                                      const int    N)
{
    std::vector<Eigen::Vector2d> zmpTrajectory;
    Eigen::Vector2d sineVector = Eigen::Vector2d::Zero();
    double t = 0;
    while (t < N*tTrans) {
        double tmp = (-feetSeparation/2)*std::sin((2*tTrans/M_PI)*t) - feetSeparation/2;
        sineVector(1) = tmp;
        zmpTrajectory.push_back(sineVector);
        t = t + timeStep/1000;
    }
    
    return zmpTrajectory;
}

bool WalkingClient::getFeetSeparation(Eigen::Vector3d &sep) {
    Eigen::Vector3d lFootPosition = this->model->getSegmentPosition("l_foot").getTranslation();
    Eigen::Vector3d rFootPosition = this->model->getSegmentPosition("r_foot").getTranslation();
    sep = (rFootPosition - lFootPosition).cwiseAbs();
    return true;
}

bool WalkingClient::publishZMPError(Eigen::Vector2d &zmpError) {
    yarp::os::Bottle &output = _zmpPort.prepare();
    output.clear();
    output.addDouble(zmpError(0));
    output.addDouble(zmpError(1));
    _zmpPort.write();
    return true;
}
