#include "walking-client/WalkingClient.h"

using namespace Eigen;

WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod),
_zmpParams(std::make_shared<ZmpControllerParams>(1e-4,
                                                 1.5e-4,
                                                 1e-6,
                                                 5e-4,
                                                 model->getMass(),
                                                 (model->getCoMPosition()).operator()(2),
                                                 9.8,
                                                 (double) loopPeriod/1000)),
_zmpController(std::make_shared<ZmpController>(loopPeriod, modelPtr, _zmpParams)),
_zmpPreviewParams(std::make_shared<ZmpPreviewParams>(3,
                                                     (model->getCoMPosition()).operator()(2),
                                                     1e-3,
                                                     0.5,
                                                     0.5)),
_rawLeftFootWrench(Eigen::VectorXd::Zero(6)),
_rawRightFootWrench(Eigen::VectorXd::Zero(6)),
_globalZMP(Eigen::Vector2d::Zero()),
_isTestRun(false)
{

}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

bool WalkingClient::configure(yarp::os::ResourceFinder &rf) {
    // Client name
    if (!rf.check("name")) {
        OCRA_WARNING("Option 'name' was not specified. Default is the name of this module, i.e. walking-client");
        _clientName = "walkingClient";
    } else { 
        _clientName = rf.find("name").asString();
        OCRA_INFO("Client name is: " << _clientName);
    }
    // Thread period
    if (!rf.check("robot")) {
        OCRA_ERROR("Option 'robot' was not specified. Critical error. ");
        return false;
    } else {
        _robot = rf.find("robot").asString();
    }
    // Running the client for tests?
    if (!rf.check("runForTests")) {
        OCRA_WARNING("Option 'runForTests' was not found. Using FALSE as default");
        _isTestRun = false;
    } else {
        _isTestRun = rf.find("runForTests").asBool();
    }

    // Find ZMP Controller paramaters
    if (!rf.check("ZMP_CONTROLLER_PARAMS")) {
        OCRA_WARNING("Group of options ZMP_CONTROLLER_PARAMS was not found, using default parameters kpx = 1e-4, kpy = 1.5e-4, kdx = 1e-6, kdy = 5e-4, g 9.8");
    } else {
        yarp::os::Property zmpControllerParamsGroup;
        zmpControllerParamsGroup.fromString(rf.findGroup("ZMP_CONTROLLER_PARAMS").tail().toString());
        _zmpParams->kfx = zmpControllerParamsGroup.find("kfx").asDouble();
        _zmpParams->kfy = zmpControllerParamsGroup.find("kfy").asDouble();
        _zmpParams->kdx = zmpControllerParamsGroup.find("kdx").asDouble();
        _zmpParams->kdy = zmpControllerParamsGroup.find("kdy").asDouble();
        _zmpParams->g = zmpControllerParamsGroup.find("g").asDouble();
        OCRA_INFO(">> [ZMP_CONTROLLER_PARAMS]: \n " << zmpControllerParamsGroup.toString().c_str());
    }

    yarp::os::Time::delay(2);
    
    // Find ZMP Preview Controller parameters
    if (!rf.check("ZMP_PREVIEW_CONTROLLER_PARAMS")) {
        OCRA_WARNING("Group of options ZMP_PREVIEW_CONTROLLER_PARAMS was not found, using default parameters Nc=3, nu=, nw=, nb= ");
    } else {
        yarp::os::Property zmpPreviewControllerParamsGroup;
        zmpPreviewControllerParamsGroup.fromString(rf.findGroup("ZMP_PREVIEW_CONTROLLER_PARAMS").tail().toString());
        _zmpPreviewParams->cz = this->model->getCoMPosition().operator()(2);
        _zmpPreviewParams->Nc = zmpPreviewControllerParamsGroup.find("Nc").asInt();
        _zmpPreviewParams->nw = zmpPreviewControllerParamsGroup.find("nw").asDouble();
        _zmpPreviewParams->nb = zmpPreviewControllerParamsGroup.find("nb").asDouble();
        _zmpPreviewParams->nu = zmpPreviewControllerParamsGroup.find("nu").asDouble();
        OCRA_INFO(">> [ZMP_PREVIEW_CONTROLLER_PARAMS]: \n " << zmpPreviewControllerParamsGroup.toString().c_str() << " cz: " << _zmpPreviewParams->cz);
    }    
    
    // Create zmpPreviewController object
   _zmpPreviewController = std::make_shared<ZmpPreviewController>((double) this->getExpectedPeriod(), _zmpPreviewParams);

    // Find TESTS_GENERAL_PARAMETERS
    if (!rf.check("TESTS_GENERAL_PARAMETERS")) {
            OCRA_WARNING("Group of options TESTS_GENERAL_PARAMETERS was not found, using default parameters _zmpTestType " << _zmpTestType << "_homeDataDir: " << _homeDataDir);
            _zmpTestType = ZmpTestType::ZMP_VARYING_REFERENCE;
            _homeDataDir = "/home/";
    } else {
            yarp::os::Property testsGeneralParamsGroup;
            testsGeneralParamsGroup.fromString(rf.findGroup("TESTS_GENERAL_PARAMETERS").tail().toString());
            _zmpTestType = (ZmpTestType) testsGeneralParamsGroup.find("type").asInt();
            _homeDataDir = testsGeneralParamsGroup.find("homeDataDir").asString();
            OCRA_INFO(">> [TESTS_GENERAL_PARAMETERS]: \n " << testsGeneralParamsGroup.toString().c_str());
    }

    // Find COM_LIN_VEL_CONSTANT_REFERENCE
    if (!rf.check("COM_LIN_VEL_CONSTANT_REFERENCE")) {
        OCRA_WARNING("Group COM_LIN_VEL_CONSTANT_REFERENCE was not found, using default parameters comYConstVel=0.01, stopTimeConstComVel 2");
        _comYConstVel = 0.01;
        _stopTimeConstComVel = 2;
    } else {
        yarp::os::Property comLinVelConstantReferenceGroup;
        comLinVelConstantReferenceGroup.fromString(rf.findGroup("COM_LIN_VEL_CONSTANT_REFERENCE").tail().toString());
        _comYConstVel = comLinVelConstantReferenceGroup.find("comYConstVel").asDouble();
        _stopTimeConstComVel = comLinVelConstantReferenceGroup.find("stopTimeConstComVel").asDouble();
        OCRA_INFO(">> [COM_LIN_VEL_CONSTANT_REFERENCE]: \n " << comLinVelConstantReferenceGroup.toString().c_str());
    }

    // Find ZMP_CONSTANT_REFERENCE group
    if (!rf.check("ZMP_CONSTANT_REFERENCE")) {
        OCRA_WARNING("Group ZMP_CONSTANT_REFERENCE was not found, using default parameters zmpYConstRef=-0.10, stopTimeConstZmp=3");
        _zmpYConstRef = 0.10;
        _stopTimeConstZmp = 3;
    } else {
        yarp::os::Property zmpConstantReferenceGroup;
        zmpConstantReferenceGroup.fromString(rf.findGroup("ZMP_CONSTANT_REFERENCE").tail().toString());
        _zmpYConstRef = zmpConstantReferenceGroup.find("zmpYConstRef").asDouble();
        _stopTimeConstZmp = zmpConstantReferenceGroup.find("stopTimeConstZmp").asDouble();
        OCRA_INFO(">> [ZMP_CONSTANT_REFERENCE]: \n " << zmpConstantReferenceGroup.toString().c_str());
    }

    // Find ZMP_VARYING_REFERENCE
    if (!rf.check("ZMP_VARYING_REFERENCE")) {
        OCRA_WARNING("Group ZMP_VARYING_REFERENCE was not found, using default parameters tTrans=3, numberOfTransitions=6, amplitudeFraction=3");
        _tTrans = 3;
        _numberOfTransitions = 6;
        _amplitudeFraction = 3;
        _stopTimeVaryingZmp = 12;
    } else {
        yarp::os::Property zmpVaryingReferenceGroup;
        zmpVaryingReferenceGroup.fromString(rf.findGroup("ZMP_VARYING_REFERENCE").tail().toString());
        _tTrans = zmpVaryingReferenceGroup.find("tTrans").asDouble();
        _numberOfTransitions = zmpVaryingReferenceGroup.find("numberOfTransitions").asInt();
        _amplitudeFraction = zmpVaryingReferenceGroup.find("amplitudeFraction").asInt();
        _stopTimeVaryingZmp = zmpVaryingReferenceGroup.find("stopTimeVaryingZmp").asDouble();
        OCRA_INFO(">> [ZMP_VARYING_REFERENCE]: \n " << zmpVaryingReferenceGroup.toString().c_str());
    }
    

    return true;
}

bool WalkingClient::initialize()
{
    // Connect to feet wrench ports
    bool ok = portWrenchLeftFoot.open(composePortName("left_foot/wrench:i"));
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/left_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        std::string src = std::string("/"+_robot+"/left_foot/analog:o");
        if (!yarp::os::Network::connect(src, portWrenchLeftFoot.getName().c_str())) {
            OCRA_ERROR("Impossible to connect to " << src);
            return false;
        }
    }
    ok = portWrenchRightFoot.open(composePortName("right_foot/wrench:i"));
    if (!ok) {
        OCRA_ERROR("Impossible to open /right_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        std::string src = std::string("/"+_robot+"/right_foot/analog:o");
        if (!yarp::os::Network::connect(src, portWrenchRightFoot.getName().c_str()) ) {
            OCRA_ERROR("Impossible to connect to " << src);
            return false;
        }
    }


    // CoM TaskConnection object. This is the task object through which the CoM velocity will be set by this controller.
    std::string comTaskName("ComTask");
    _comTask = std::make_shared<ocra_recipes::TaskConnection>(comTaskName);
    _comTask->openControlPorts();


    // For testing and tuning purposes, generate ZMP trajectory that moves from one foot to the other.
    Eigen::Vector3d sep; sep.setZero();
    getFeetSeparation(sep);
    double feetSeparation = sep(1);
    int period = this->getExpectedPeriod();
    _zmpTrajectory = generateZMPTrajectoryTEST(_tTrans, feetSeparation, period, _amplitudeFraction, _numberOfTransitions);

    // If ZMP tests are being run, publish ZMP on port
//     if (_isTestRun) {
//         _zmpPort.open(composePortName("zmpError:o"));
//         _dcomErrorPort.open(composePortName("dcomError:o"));
//         _dComDesPort.open(composePortName("dComDesired:o"));
//         _dComCurPort.open(composePortName("dComCurrent:o"));
//         _zmpDesPort.open(composePortName("zmpDesired:o"));
//         _zmpCurPort.open(composePortName("zmpCurrent:o"));
//         _comCurrent.open(composePortName("comCurrent:o"));
//     }

    Eigen::Vector3d initialCOMPosition = this->model->getCoMPosition();
    _zmpParams->cz = initialCOMPosition(2);
    _previousCOM = initialCOMPosition.topRows(2);
    _hkkPrevious.setZero(6);
    _firstLoop = true;
    
    OCRA_INFO("Initialization is over");
    return true;
}

void WalkingClient::release()
{
//     if (_isTestRun)
//     {
//         _zmpPort.close();
//         _dcomErrorPort.close();
//         _dComDesPort.close();
//         _zmpDesPort.close();
//         _zmpCurPort.close();
//         _comCurrent.close();
//         _ddcomCurrent.close();
//         _ddcomFromZMP.close();
//     }
}

void WalkingClient::loop()
{
    // Read measurements
    readFootWrench(LEFT_FOOT, _rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, _rawRightFootWrench);

    // Compute ZMP
    _zmpController->computeGlobalZMPFromSensors(_rawLeftFootWrench, _rawRightFootWrench, _globalZMP);

    if (_isTestRun) {
        // Specify type of zmp test
        performZMPTest(_zmpTestType);
    } else {
        OCRA_INFO("ZMP Preview test");
    // Compute ZMPPreviewController optimal input
        performZMPPreviewTest(_zmpTestType);
    }
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

std::vector<Eigen::Vector2d> WalkingClient::generateZMPTrajectoryTEST(double tTrans,
                                                                      double feetSeparation,
                                                                      double timeStep,
                                                                      int    amplitudeFraction,
                                                                      int    N)
{
    std::vector<Eigen::Vector2d> zmpTrajectory;
    Eigen::Vector2d sineVector = Eigen::Vector2d::Zero();
    double t = 0;
    double tmp;
    while (t < N*tTrans) {
        tmp = (-feetSeparation/amplitudeFraction)*std::sin((M_PI/(2*tTrans))*t) - feetSeparation/2;
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


bool WalkingClient::publish3dQuantity(yarp::os::BufferedPort<yarp::os::Bottle> &port, Eigen::Vector3d &value) {
    yarp::os::Bottle &output = port.prepare();
    output.clear();
    output.addDouble(value(0));
    output.addDouble(value(1));
    output.addDouble(value(2));
    port.write();
    return true;
}

void WalkingClient::transformStdVectorToEigenVector(std::vector<Eigen::Vector2d> &fullTraj, int from, int Nc, Eigen::VectorXd &output) 
{
//     if (output.RowsAtCompileTime != 2*Nc) {
//         OCRA_ERROR("The output vector has the wrong size!");
//         this->askToStop();
//     }
    
    int k = 0;
    for (int i=from; i<=from+Nc-1; i++) {
        output(k) = fullTraj[i].operator()(0);
        output(k+1) = fullTraj[i].operator()(1); 
        k = k + 2;
    }
       
}

void WalkingClient::performZMPPreviewTest(ZmpTestType type)
{
    OCRA_INFO("Performing zmp preview test");
    static double timeInit = yarp::os::Time::now();
    Eigen::Vector2d zmpReference;
    // Vector of the next Nc references. Size is 2*Nc because every zmp reference is bidimensional
    Eigen::VectorXd zmpRef(2*_zmpPreviewParams->Nc); 
    zmpRef.setZero();
    // For the test the reference com velocity is set to zero. Only zmp references are considered.
    Eigen::VectorXd comVelRef(2*_zmpPreviewParams->Nc);
    comVelRef.setZero();
    Eigen::VectorXd optimalU(2*_zmpPreviewParams->Nc);
    // Retrieve current COM state 
    Eigen::VectorXd hk(6);
    Eigen::Vector2d h = this->model->getCoMPosition().topRows(2);
    Eigen::Vector2d dh = this->model->getCoMVelocity().topRows(2);
    Eigen::Vector2d ddh; ddh.setZero();
    hk.head<2>() = h;
    hk.segment<2>(2) = dh;
    hk.tail<2>() = ddh;
    OCRA_INFO("hk is: " << hk.transpose());
    Eigen::VectorXd hkk(6); hkk.setZero();
    //TODO: This should be passed to the initialization method of the walkingClient.
    // Initial value of hkkPrevious equal to the initial state hk
    if (_firstLoop) {
        _hkkPrevious = hk;
        _firstLoop = false;
    }
    Eigen::Vector2d pk; pk.setZero();
    Eigen::Vector2d intddhkk; intddhkk.setZero();
    Eigen::Vector2d inthkk; inthkk.setZero();
    Eigen::Vector2d dhd; dhd.setZero();
    Eigen::Vector2d intComPosition; intComPosition.setZero();
    double tnow = yarp::os::Time::now();

    // Transform the following Nc zmp references from the current time step in the std::vector container to a single Eigen::VectorXd
    
    static int el = 0;
    
    if (type == ZMP_VARYING_REFERENCE) {
            zmpReference = _zmpTrajectory[el];
            // Compute optimal input in preview window for the next Nc zmp references
            // [CHECKED]
            transformStdVectorToEigenVector(_zmpTrajectory, el, _zmpPreviewParams->Nc, zmpRef);
            
//             OCRA_INFO("Zmp Reference for " << _zmpPreviewParams->Nc << "-sized window is: \n" << zmpRef );
//             OCRA_INFO("Com Vel Reference: " << comVelRef.transpose());
//             OCRA_INFO("Current com state: " << hk.transpose());
            // [CHECKED]
            _zmpPreviewController->computeOptimalInput(zmpRef, comVelRef, _hkkPrevious, optimalU);
            
//             OCRA_INFO("Optimal input: " << optimalU.transpose() );
            /** Only using the first input computed by the preview controller;
             * This input must now be integrated (since it's just the optimal com jerk)
             * [CHECKED]
             */
            _zmpPreviewController->integrateComJerk(optimalU.topRows(2), _hkkPrevious, hkk);
            
//             OCRA_INFO("Integrated state: " << hkk.transpose());
            /** Using the zmp cart model, the instantaneous zmp trajectory can now be
             * computed. For this we'll pass the current com state hk and the integrated 
             * optimal com acceleration.
             */
             _hkkPrevious = hkk;
             intddhkk = hkk.tail<2>();
             inthkk = hkk.head<2>();
            _zmpPreviewController->tableCartModel(inthkk, intddhkk, pk);
            
//             OCRA_INFO("Preview zmp: " << pk.transpose());
            /** The zmp controller will now use this instantaneous value as reference. 
             * However, for this reference, a corresponding desired com position and 
             * velocity are computed to reduced the zmp tracking error.
             */
             // First, compute the corresponding desired velocity
            _zmpController->computehd(_globalZMP, pk, dhd);
            
//             OCRA_INFO("COM velocity reference: " << dhd.transpose());
             // With the previous velocity, compute the corresponding desired com position
            _zmpController->computeh(_previousCOM, dhd, intComPosition);
//             OCRA_INFO("Integrated COM Position: " << intComPosition.transpose());
            
            _previousCOM = intComPosition;
            
            /** Apply control 
             *  The ZMP Controller is in charge of computing the control
             */
            ocra::TaskState desiredState = _zmpController->computeControl(intComPosition, dhd);
//             OCRA_INFO("Going to apply the following state: " << desiredState);
            _comTask->setDesiredTaskStateDirect(desiredState);
            if ( el < _zmpTrajectory.size() )
                el++;
            else
                this->askToStop();
    }
    
    // Read actual state
    Eigen::Vector3d currentComLinVel;
    currentComLinVel = _comTask->getTaskState().getVelocity().getLinearVelocity();
//     OCRA_INFO("Current com lin vel " << currentComLinVel.transpose());

    Eigen::Vector3d currentComPos;
    currentComPos = _comTask->getTaskState().getPosition().getTranslation();
//     OCRA_INFO("Current com position " << currentComPos.transpose());

//     Ref. Linear Velocity
    Eigen::Vector3d refLinVel( dhd(0), dhd(1), 0 );
    
//     Write to file for plotting
    tnow = yarp::os::Time::now() - timeInit;
    std::string homeDir = std::string(_homeDataDir + "zmpPreviewController/");
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, refLinVel).finished(), std::string(homeDir + "refLinVel.txt") ,true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComLinVel).finished(), std::string(homeDir + "currentComLinVel.txt"),true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, intComPosition).finished(), std::string(homeDir + "intComPositionRef.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComPos).finished(), std::string(homeDir + "currentComPos.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, _globalZMP).finished(), std::string(homeDir + "currentZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, pk).finished(), std::string(homeDir + "previewedZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, zmpReference).finished(), std::string(homeDir + "referenceZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, optimalU.topRows(2)).finished(), std::string(homeDir + "optimalInput.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, inthkk).finished(), std::string(homeDir + "integratedCOM.txt"), true);
}


void WalkingClient::performZMPTest(ZmpTestType type) {

    static int el = 0;
    static double timeInit = yarp::os::Time::now();
    double tnow = yarp::os::Time::now();

    Eigen::Vector2d constantZMPRef;
    Eigen::Vector3d constantRefLinVel;
    Eigen::Vector2d zmpReference;
    Eigen::Vector2d dhd; dhd.setZero();
    Eigen::Vector2d ddh; ddh.setZero();
    Eigen::Vector2d hdes; hdes.setZero();
    Eigen::Vector2d intComPosition; intComPosition.setZero();


    switch (type) {
        case ZMP_CONSTANT_REFERENCE:
            zmpReference  << 0, _zmpYConstRef, 0;
            _zmpController->computehd(_globalZMP, zmpReference, dhd);
            if (tnow - timeInit > _stopTimeConstZmp) {
                OCRA_WARNING("Stopping client for ZMP_CONSTANT_REFERENCE test");
                this->askToStop();
            }
            break;
        case COM_LIN_VEL_CONSTANT_REFERENCE:
            constantRefLinVel << 0, _comYConstVel, 0;
            // Stop sending this constant reference after 2 seconds.
            if (tnow - timeInit > _stopTimeConstComVel) {
                constantRefLinVel(1) = 0.0;
                OCRA_WARNING("Stopping client for COM_LIN_VEL_CONSTANT_REFERENCE");
                this->askToStop();
            }
            dhd = constantRefLinVel.topRows(2);
            break;
        case ZMP_VARYING_REFERENCE:
            zmpReference = _zmpTrajectory[el];
            _zmpController->computehd(_globalZMP, zmpReference, dhd);
            if ( el < _zmpTrajectory.size() ){
                el++;
            }
            if (tnow - timeInit > _stopTimeVaryingZmp) {
                OCRA_WARNING("Stopping client for ZMP_VARYING_REFERENCE test");
                this->askToStop();
            }
            break;
        default:
            break;
    }

    // Given a constant zmp reference, compute the corresponding COM velocity. Transparent to any test.
    // Integrate from this velocity the com position
    _zmpController->computeh(_previousCOM, dhd, intComPosition);
    _previousCOM = intComPosition;
    _zmpController->computehdd((Eigen::Vector3d() << intComPosition, 0).finished(), zmpReference, ddh);


    // Prepare desired com state
    // Desired com velocity
    Eigen::Vector3d zeroAngVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d refLinVel( dhd(0), dhd(1), 0 );
    Eigen::Twistd refComVelocity( zeroAngVel, refLinVel );

    // Desired com acceleration
//    Eigen::Vector3d zeroAngAcc = Eigen::Vector3d::Zero();
//    Eigen::Vector3d refLinAcc( ddh(0), ddh(1), 0 );
//    Eigen::Twistd refComAcceleration( zeroAngAcc, refLinAcc );

    // Desired com position
    Eigen::Vector3d position;
    position(0) = intComPosition(0);
    position(1) = intComPosition(1);
    position(2) = _zmpParams->cz;
    Eigen::VectorXd displacement(7);
    Eigen::VectorXd orientation(4);
    orientation << 1.0, 0.0, 0.0, 0.0;
    displacement << position, orientation;
    Eigen::Displacementd refComPosition = ocra::util::eigenVectorToDisplacementd(displacement);

    // Apply CONTROL
    _desiredComState.setVelocity(refComVelocity);
    _desiredComState.setPosition(refComPosition);
    _comTask->setDesiredTaskStateDirect(_desiredComState);
    
    // Read actual state
    Eigen::Vector3d currentComLinVel;
    currentComLinVel = _comTask->getTaskState().getVelocity().getLinearVelocity();

    Eigen::Vector3d currentComPos;
    currentComPos = _comTask->getTaskState().getPosition().getTranslation();

    // Write to file for plotting
    tnow = yarp::os::Time::now() - timeInit;
    std::string homeDir = std::string(_homeDataDir);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, refLinVel).finished(), std::string(homeDir + "refLinVel.txt") ,true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComLinVel).finished(), std::string(homeDir + "currentComLinVel.txt"),true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, intComPosition).finished(), std::string(homeDir + "intComPositionRef.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComPos).finished(), std::string(homeDir + "currentComPos.txt"), true);

}

 std::string WalkingClient::composePortName(std::string portName) {
    std::string tmp;
    tmp = std::string("/" + _clientName + "/" + portName);
    return tmp;
}
