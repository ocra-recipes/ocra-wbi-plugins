#include "walking-client/WalkingClient.h"


using namespace Eigen;

WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod),
_zmpPreviewParams(std::make_shared<ZmpPreviewParams>(50,
                                                     50,
                                                     (model->getCoMPosition()).operator()(2),
                                                     1e-6,
                                                     0.0,
                                                     1.0)),
_rawLeftFootWrench(Eigen::VectorXd::Zero(6)),
_rawRightFootWrench(Eigen::VectorXd::Zero(6)),
_globalZMP(Eigen::Vector2d::Zero()),
_isTestRun(false),
_currentlyStepping(false),
_waitBeforeNextStep(false),
_currentStepIndex(0)
{

}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

void WalkingClient::printHelp()
{
    std::cout << "\n\n\n\n\n" << std::endl;
    std::cout << "=== walking-client Options List === \n" << std::endl;
    std::cout << "[TESTS_GENERAL_PARAMETERS]" << std::endl;
    std::cout << "type: \t [0] ZMP_CONSTANT_REFERENCE\n \
                        \t [1] ZMP_VARYING_REFERENCE\n \
                        \t [2] COM_LIN_VEL_CONSTANT_REFERENCE" << std::endl;
    std::cout << "\n\n\n\n\n" << std::endl;

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
    // Robot name
    if (!rf.check("robot")) {
        OCRA_ERROR("Option 'robot' was not specified. Critical error. ");
        return false;
    } else {
        _robot = rf.find("robot").asString();
    }
    // Running the client for tests?
    if (!rf.check("test")) {
        OCRA_WARNING("Option 'test' was not found. If you wanted to perform some test, please pass the option --test zmpPreview or --test zmpController");
        _isTestRun = false;
    } else {
        _isTestRun = true;
        _testType = rf.find("test").asString();
    }

    //WARNING: This innocent delay is important here to ensure that the model has been properly loaded
    yarp::os::Time::delay(2);

    // Find ZMP Preview Controller parameters
    findZMPPreviewControllerParams(rf);
    // Find TESTS_GENERAL_PARAMETERS
    findGeneralTestsParams(rf);
    // Find COM_LIN_VEL_CONSTANT_REFERENCE
    findCOMLinVelConstRefParams(rf);
    // Find ZMP_CONSTANT_REFERENCE group
    findZMPConstRefParams(rf);
    // Find single step test parameters
    findSingleStepTestParams(rf);
    // Find stepping test parameters
    findSteppingTestParams(rf);
    // Find ZMP_VARYING_REFERENCE
    findZMPVaryingReferenceParams(rf);
    // Find MIQP Parameters
    findMIQPParams(rf);

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

    _period = this->getExpectedPeriod();


     // Prepare feet cartesian tasks
    _stepController = std::make_shared<StepController>(_period, this->model);
    _stepController->initialize();


    // CoM TaskConnection object. This is the task object through which the CoM acceleration will be set by this controller.
    std::string comTaskName("ComTask");
    _comTask = std::make_shared<ocra_recipes::TaskConnection>(comTaskName);
    _comTask->openControlPorts();


    // For testing and tuning purposes, generate ZMP trajectory that moves from one foot to the other.
    Eigen::Vector3d sep; sep.setZero();
    getFeetSeparation(sep);
    double feetSeparation = sep(1);
    if (_zmpTestType == ZMP_VARYING_REFERENCE) {
        _zmpTrajectory = generateZMPTrajectoryTEST(_tTrans, feetSeparation, _period, _amplitudeFraction, _numberOfTransitions);
    } else {
        if (_zmpTestType == ZMP_CONSTANT_REFERENCE) {
            OCRA_WARNING("A ZMP Step Reference trajectory will be created");
            _zmpTrajectory = generateZMPStepTrajectoryTEST(feetSeparation, _period, _trajectoryDurationConstZmp, _riseTimeConstZmp, _zmpYConstRef);
        }
    }

    if (!_testType.compare("singleStepTest")) {
        OCRA_INFO("A single step zmp trajectory will be created");
        OCRA_WARNING("Feet separation: " << sep);
        _singleStepTrajectory = generateZMPSingleStepTrajectory(_period, feetSeparation);
    }
    if (!_testType.compare("steppingTest")) {
        OCRA_INFO("A single step zmp trajectory will be created");
        OCRA_WARNING("Feet separation: " << sep);
        generateStepPattern();
        _steppingTrajectory = generateZMPSteppingTrajectory();
        // for (auto v : _steppingTrajectory)
        // {
        //     std::cout << v.transpose() << std::endl;
        // }
    }

    Eigen::Vector3d initialCOMPosition = this->model->getCoMPosition();
    _previousCOM = initialCOMPosition.topRows(2);
    _previousCOMVel = this->model->getCoMVelocity().topRows(2);
    _hkkPrevious.setZero(6);
    _firstLoop = true;

    // Read measurements
    readFootWrench(LEFT_FOOT, _rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, _rawRightFootWrench);

    // Compute ZMP
    _zmpPreviewController->computeGlobalZMPFromSensors(_rawLeftFootWrench, _rawRightFootWrench, this->model, _globalZMP);

    // Vector of the next Np references. Size is 2*Np because every zmp reference is bidimensional
    zmpRefInPreviewWindow = Eigen::VectorXd(2*_zmpPreviewParams->Np);
    zmpRefInPreviewWindow.setZero();
    // For the test the reference com velocity is set to zero. Only zmp references are considered.
    comVelRefInPreviewWindow = Eigen::VectorXd(2*_zmpPreviewParams->Np);
    comVelRefInPreviewWindow.setZero();
    // Allocation of optimal input vector
    optimalU = Eigen::VectorXd(2*_zmpPreviewParams->Nc);

    //FIXME: Is this necessary?
    if (_testType.compare("steppingTest")) {
        // Set task's Kp and Kd to 0 from the client, since this task will receive pure accelerations
    }

    // Start MIQPController thread
    // FIXME: Dummy initial COM velocity references. They're both ramps, to test in the preview window.
    Eigen::MatrixXd comStateRef(100*_miqpParams.N, 6);
    double dCoMxRef = _miqpParams.dCoMxRef; 
    double dCoMyRef = _miqpParams.dCoMyRef;
    Eigen::VectorXd comRefToReplicate(6); comRefToReplicate << _previousCOM, dCoMxRef, dCoMyRef, 0, 0;
    comStateRef = comRefToReplicate.transpose().replicate(100*_miqpParams.N,1);
    OCRA_WARNING("dCoMxRef is: " << dCoMxRef);
    OCRA_WARNING("dCoMyRef is: " << dCoMyRef);
    //FIXME: In this way I'm creating a velocity ramp only in the first preview window! 
    comStateRef(0,2) = 0;
    comStateRef(1,2) = 0;
    comStateRef(2,2) = 0.33*_miqpParams.dCoMxRef;
    comStateRef(3,2) = 0.66*_miqpParams.dCoMxRef;
    comStateRef(_miqpParams.N-3,2) = 0.66*_miqpParams.dCoMxRef;
    comStateRef(_miqpParams.N-2,2) = 0.33*_miqpParams.dCoMxRef;
    comStateRef(_miqpParams.N-1,2) = 0;
    comStateRef(_miqpParams.N, 2) = 0;
    
    comStateRef(0,3) = 0;
    comStateRef(1,3) = 0;
    comStateRef(2,3) = 0.33*dCoMyRef;
    comStateRef(3,3) = 0.66*dCoMyRef;
    comStateRef(_miqpParams.N-3,3) = 0.66*dCoMyRef;
    comStateRef(_miqpParams.N-2,3) = 0.33*dCoMyRef;
    comStateRef(_miqpParams.N-1,3) = 0;
    comStateRef(_miqpParams.N, 3) = 0;

    OCRA_INFO(">>> FIRST REFS: \n");
    OCRA_INFO(comStateRef.block(0,0,_miqpParams.N,6));
    // Current iteration
    _k = 1;
    // Allocate space for optimal horizon of solutions
    int Nw;
    // Starting sample point in the original solution from which everything will be copied as it is.
    unsigned int fromSample = (unsigned int) std::ceil( (2*_miqpParams.dtThread - _miqpParams.dt)/_miqpParams.dt ) + 1; 
    Nw = _miqpParams.N - fromSample + 1 + 2;
    OCRA_ERROR("Space allocated for preview window of the walking-client: " << Nw);
    this->_X_kn.resize(Nw*INPUT_VECTOR_SIZE);
    this->_t_kn.resize(Nw);
    _miqpController = std::make_shared<MIQPController>(_miqpParams, this->model, this->_stepController, comStateRef);
    _miqpController->start();
    // Don't run this thread before the MIQPController class has finished initializing
    while (!_miqpController->isRunning()) {
        OCRA_WARNING("Waiting for MIQPController thread to start");
    }
    OCRA_INFO("Initialization is over");
    return true;
}

void WalkingClient::release()
{
     // Set task's Kp and Kd to initial values before starting the client
//     _comTask->setStiffness(30);
//     _comTask->setDamping(5);
    
    _miqpController->stop();
    _stepController->stop();
}

void WalkingClient::loop()
{
    if (_isTestRun) {
       if (!_testType.compare("zmpPreview")) {
           // Track a zmp trajectory using a zmp preview controller
           performZMPPreviewTest(_zmpTestType);
       } else if (!_testType.compare("steppingTest")) {
           this->steppingTest();
       } else {
           if (!_testType.compare("singleStepTest")) {
           // Performing one single step with the right foot
               performSingleStepTest();
           } else {
               OCRA_ERROR("You want to perform a zmp test, but zmpPreview was not found as value for the option 'test'. Please try again... ");
               this->askToStop();
           }
       }
    } else {
       // INTERFACE THE MIQP CONTROLLER WITH THE MAIN THREAD OF THE WALKING CLIENT! THIS IS THE REAL DEAL
        if ( queryMIQPSolution(_miqpParams.dtThread, _miqpParams.dt, this->getExpectedPeriod(), _X_kn, _t_kn) ) {
            // If a new solution was retrieved from the MIQP
            OCRA_ERROR("A new solution was retrieved after " << _k << " samples of walking-client and it was: \n ");
            int Nw = (int) _X_kn.size()/INPUT_VECTOR_SIZE;
            for (unsigned int i = 0; i < Nw; i++) {
                std::cout << _X_kn.segment(i*INPUT_VECTOR_SIZE, INPUT_VECTOR_SIZE).transpose() << std::endl;
            }
            std::cout << "Time vector is: \n" << _t_kn.transpose() << std::endl;
            // INTERPOLATION TO GENERATE CoP and CoM velocity reference
            // First create vector of doubles with the times to interpolate
            std::vector<double> timeToInterp;
            for (unsigned int i = 0; i < _t_kn.size(); i++) {
                timeToInterp.push_back(_t_kn(i)/1000);
            }
            // Then create vector of points to interpolate a and b
            std::vector<Eigen::Vector2d> aToInterp;
            std::vector<Eigen::Vector2d> bToInterp;
            int samples = (int) _X_kn.size()/INPUT_VECTOR_SIZE;
            for (unsigned int i = 0; i < samples; i++) {
                aToInterp.push_back(_X_kn.segment(i*INPUT_VECTOR_SIZE,2));
                bToInterp.push_back(_X_kn.segment(i*INPUT_VECTOR_SIZE + 2,2));
            }
            std::vector<double> intTime;       // Result of interpolation for time
            std::vector<Eigen::Vector2d> intA; // Result of interpolation for a
            std::vector<Eigen::Vector2d> intB; // Result of interpolation for b
            double dt = (double) this->_period/1000;
            linearInterpolation(timeToInterp, aToInterp, dt, intTime, intA);
            linearInterpolation(timeToInterp, bToInterp, dt, intTime, intB);
            // Now that we have interpolated a and b, we can compute the corresponding interpolated CoP
            //...
        }
        _k++;
        //FIXME: Remember to remove this when using state feedback
        if (_k==7)
            this->askToStop();
    }
}

bool WalkingClient::queryMIQPSolution(const int miqpPeriod, const int miqpPreviewPeriod, const int clientPeriod, Eigen::VectorXd &preview, Eigen::VectorXd &timeVector) {
    // If current iteration _k is a multiple of miqpPeriod/clientPeriod, then a new solution from the MIQP should be ready.
    preview.setZero();
    int samplesInPreview = (int) miqpPeriod/clientPeriod;
    unsigned int fromSample = (unsigned int) std::ceil( (2*miqpPeriod - miqpPreviewPeriod)/miqpPreviewPeriod ) + 1; 
    Eigen::VectorXd tmpXkn(INPUT_VECTOR_SIZE*_miqpParams.N);
    if ( _k % samplesInPreview == 0 ) {
        _miqpController->semaphore.wait();
        _miqpController->getSolution(tmpXkn);
        _miqpController->semaphore.post();
        // Take only the first solution and copy it the first samplesInPreview of _X_kn (or &preview)
        for (unsigned int i = 0; i < _miqpParams.N; i++)
            std::cout << tmpXkn.segment(i*INPUT_VECTOR_SIZE, INPUT_VECTOR_SIZE).transpose() << std::endl;
        // Repeat the first solution twice
        _X_kn.segment(0,INPUT_VECTOR_SIZE) = tmpXkn.topRows(INPUT_VECTOR_SIZE);
        _X_kn.segment(INPUT_VECTOR_SIZE,INPUT_VECTOR_SIZE) = tmpXkn.topRows(INPUT_VECTOR_SIZE);
        // From fromSample copy every element in the preview window
        // FIXME: The first two elements will always be X_{k+1}
        _X_kn.segment(2*INPUT_VECTOR_SIZE, (_miqpParams.N - fromSample + 1)*INPUT_VECTOR_SIZE) = tmpXkn.segment((fromSample-1)*INPUT_VECTOR_SIZE, INPUT_VECTOR_SIZE*(_miqpParams.N - fromSample + 1));
        // Fill-in time vector
        timeVector(0) = 0;
        timeVector(1) = miqpPeriod - clientPeriod;
        for (unsigned int i = 0; i <= _miqpParams.N - fromSample; i++) {
            timeVector(i+2)= miqpPeriod + i*miqpPreviewPeriod;
        }
        return true;
    } else {
        return false;
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

std::vector< Eigen::Vector2d > WalkingClient::generateZMPStepTrajectoryTEST(double feetSeparation, double period, double duration, double riseTime, double constantReferenceY)
{
    std::vector<Eigen::Vector2d> zmpTrajectory;
    Eigen::Vector2d stepReference = Eigen::Vector2d::Zero();
    double t = 0;
    while (t < duration) {
        if (t < riseTime) {
            //WARNING: This minus (-) is assuming that the world reference frame is always on the left foot!
            stepReference(1) = -feetSeparation/2;
        } else {
            stepReference(1) = constantReferenceY;
        }
        zmpTrajectory.push_back(stepReference);
        t = t + period/1000;
    }

    return zmpTrajectory;
}

std::vector<Eigen::Vector2d> WalkingClient::generateZMPSingleStepTrajectory(double period, double feetSeparation) {
    std::vector<Eigen::Vector2d> zmpTrajectory;
    Eigen::Vector2d reference = Eigen::Vector2d::Zero();
    double betweenFeetY = -feetSeparation/2;
    double t = 0;
    while (t < _singleStepTestParams.totalDuration) {
        if (t < _singleStepTestParams.riseTime) {
            reference(1) = betweenFeetY;
        } else {
            if ( t > _singleStepTestParams.riseTime && t < ( _singleStepTestParams.riseTime + _singleStepTestParams.SSduration ) ) {
                Eigen::Vector3d leftFootPosition = _stepController->getLeftFootPosition();
                reference(1) = leftFootPosition(1) - _singleStepTestParams.offset;
            } /*else {
                if ( t > ( _singleStepTestParams.riseTime + _singleStepTestParams.SSduration ) ) {
                    Eigen::Vector3d rightFootPosition = _stepController->getRightFootPosition();
                    reference(1) = betweenFeetY;
                    reference(0) =  _singleStepTestParams.stepLength;;
                }
            }*/
        }
        zmpTrajectory.push_back(reference);
        t = t + period/1000;
    }

    return zmpTrajectory;

}

void WalkingClient::generateStepPattern()
{
    Eigen::Vector3d leftFootPosition = _stepController->getLeftFootPosition();
    Eigen::Vector3d rightFootPosition = _stepController->getRightFootPosition();
    _stepTargets.resize(3, _steppingTestParams.nSteps+1);
    // even steps are the left foot and odd steps are the right foot
    for (auto i=0; i<_stepTargets.cols(); ++i) {
        if (i % 2 == 0) {
            _stepTargets.col(i) = rightFootPosition;
            _stepOrder.push_back(FOOT::RIGHT_FOOT);
        } else {
            _stepTargets.col(i) = leftFootPosition;
            _stepOrder.push_back(FOOT::LEFT_FOOT);
        }
    }

    // only the x-axis is changing for the step
    double stepDisplacement = _steppingTestParams.stepLength/2.0;

    // the first step only moves forward by 1/2 the step length
    _stepTargets(0,0) += stepDisplacement;

    // the intermediate steps all move the full length
    for (auto i=1; i<_stepTargets.cols()-1; ++i) {
        _stepTargets(0,i) = _stepTargets(0,i-1) + stepDisplacement;
    }

    // the final step needs to come up to the last foot which was set down
    _stepTargets(0,_steppingTestParams.nSteps) = _stepTargets(0,_steppingTestParams.nSteps-1);
    _stepTargetDurations = Eigen::VectorXd::Constant(_stepTargets.cols(), _steppingTestParams.stepDuration);
    _stepTargetDurations.head(1)(0) = _steppingTestParams.stepDuration / 2.0;
    _stepTargetDurations.tail(1)(0) = _steppingTestParams.stepDuration / 2.0;
    std::cout << "Step Targets:\n" << _stepTargets << std::endl;
    std::cout << "Step Durations:\n" << _stepTargetDurations << std::endl;
}

std::vector<Eigen::Vector2d> WalkingClient::generateZMPSteppingTrajectory()
{
    std::vector<Eigen::Vector2d> zmpTrajectory;
    Eigen::Vector2d reference = Eigen::Vector2d::Zero();
    Eigen::Vector3d leftFootPosition = _stepController->getLeftFootPosition();
    Eigen::Vector3d rightFootPosition = _stepController->getRightFootPosition();
    Eigen::Vector3d zmpStartPosition = (leftFootPosition + rightFootPosition)/2.0;
    Eigen::MatrixXd zmpWaypoints;
    int nSteps = _stepTargets.cols();
    zmpWaypoints.resize(2, 4+2*nSteps-1);
    zmpWaypoints.col(0) = zmpStartPosition.head(2);
    zmpWaypoints.col(1) = zmpStartPosition.head(2);
    zmpWaypoints.col(2) = leftFootPosition.head(2);
    zmpWaypoints.col(3) = leftFootPosition.head(2);
    int j=4;
    for(int i=0; i<(_stepTargets.cols()-1); ++i )
    {
        zmpWaypoints.col(j) = _stepTargets.col(i).head(2);
        zmpWaypoints.col(j+1) = _stepTargets.col(i).head(2);
        j+=2;
    }
    zmpWaypoints.rightCols(1) = ((_stepTargets.col(nSteps-2) + _stepTargets.col(nSteps-1))/2.0).head(2);

    Eigen::VectorXd zmpWaypointDurations = Eigen::VectorXd::Zero(zmpWaypoints.cols()-1);
    double singleSupportDuration = _steppingTestParams.stepDuration;
    double doubleSupportDuration = _steppingTestParams.stepDuration/2.0;
    double startShiftDuration = doubleSupportDuration/2.0;

    zmpWaypointDurations(0) = singleSupportDuration;
    zmpWaypointDurations(1) = startShiftDuration;
    for (int i=2; i<zmpWaypointDurations.size(); i+=2) {
        zmpWaypointDurations(i) = singleSupportDuration;
        zmpWaypointDurations(i+1) = doubleSupportDuration;
    }
    // zmpWaypointDurations.tail(1)(0) = _steppingTestParams.stepDuration / 2.0;

    std::cout << "zmpWaypointDurations:\n" << zmpWaypointDurations.transpose() << std::endl;
    std::cout << "zmpWaypoints:\n" << zmpWaypoints << std::endl;

    ocra::LinearInterpolationTrajectory traj;
    traj.setWaypoints(zmpWaypoints);
    traj.setDuration(zmpWaypointDurations);
    int period = this->getExpectedPeriod();
    Eigen::MatrixXd zmpTrajectoryMatrixTmp = traj.getFullTrajectory(period/1000.);
    Eigen::MatrixXd zmpTrajectoryMatrix = zmpTrajectoryMatrixTmp.leftCols(2).transpose();
    for (int i=0; i<zmpTrajectoryMatrix.cols(); ++i)
    {
        zmpTrajectory.push_back(zmpTrajectoryMatrix.col(i));
    }

    return zmpTrajectory;

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
    OCRA_INFO("ZMP Trajectory Generated");
    return zmpTrajectory;
}

bool WalkingClient::getFeetSeparation(Eigen::Vector3d &sep) {
    Eigen::Vector3d lFootPosition = this->model->getSegmentPosition("l_sole").getTranslation();
    Eigen::Vector3d rFootPosition = this->model->getSegmentPosition("r_sole").getTranslation();
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

void WalkingClient::transformStdVectorToEigenVector(std::vector<Eigen::Vector2d> &fullTraj, int from, int Np, Eigen::VectorXd &output)
{
    int k = 0;
    for (int i=from; i<=from+Np-1; i++) {
        output(k) = fullTraj[i].operator()(0);
        output(k+1) = fullTraj[i].operator()(1);
        k = k + 2;
    }
}

void WalkingClient::performZMPPreviewTest(ZmpTestType type)
{
    //TODO: Pass most of this shit to the initialization method
    static double timeInit = yarp::os::Time::now();
    static double tnow = yarp::os::Time::now() - timeInit;
    static int el = 0;

    Eigen::Vector2d zmpReference;
    // Retrieve current COM state
    Eigen::VectorXd hk(6);
    hk.head<2>() = this->model->getCoMPosition().topRows(2);
    hk.segment<2>(2) = this->model->getCoMVelocity().topRows(2);
    hk.tail<2>() = this->model->getCoMAcceleration().topRows(2);
    //TODO: Try to get the task state instead of pos, vel and acc individually.
    //_comTask->getTaskState();
    Eigen::VectorXd hkk(6); hkk.setZero();
    Eigen::Vector2d pk; pk.setZero();
    Eigen::Vector2d intddhkk; intddhkk.setZero();
    Eigen::Vector2d inthkk; inthkk.setZero();
    Eigen::Vector2d intdhkk; intdhkk.setZero();

    zmpReference = _zmpTrajectory[el];

    // Transform the following Np zmp references from the current time step in the std::vector container to a single Eigen::VectorXd
    transformStdVectorToEigenVector(_zmpTrajectory, el, _zmpPreviewParams->Np, zmpRefInPreviewWindow);

    // Compute optimal input in preview window for the next Np zmp references
    _zmpPreviewController->computeOptimalInput(zmpRefInPreviewWindow, comVelRefInPreviewWindow, hk, optimalU);

    // Only using the first input computed by the preview controller;
    // This input must now be integrated (since it's just the optimal com jerk)
    _zmpPreviewController->integrateCom(optimalU.topRows(2), hk, hkk);

    // Using the zmp cart model, the instantaneous zmp trajectory can now be
    // computed. For this we'll pass the current full com state hk.
    inthkk = hkk.head<2>();
    intdhkk = hkk.segment<2>(2);
    intddhkk = hkk.tail<2>() + 0.010*optimalU.topRows(2);
    hkk.tail<2>() = intddhkk;
    _zmpPreviewController->tableCartModel(hkk, pk);

    // Apply control
    // Prepare the desired com state and apply control!
    prepareAndsetDesiredCoMTaskState(hkk, true);

    Eigen::Vector3d currentComPos;
    currentComPos = _comTask->getTaskState().getPosition().getTranslation();

    Eigen::Vector3d currentAcceleration;
    currentAcceleration = this->model->getCoMAcceleration();

    // Read Force/Torque measurements
    readFootWrench(LEFT_FOOT, _rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, _rawRightFootWrench);

    // Compute ZMP
    _zmpPreviewController->computeGlobalZMPFromSensors(_rawLeftFootWrench, _rawRightFootWrench, this->model, _globalZMP);

    // Write to file for plotting
    //TODO: Watch out! if the thread doesn't respect the desired period, then your plots will look horizontally scaled!
    tnow = tnow + this->getEstPeriod()/1000;
    std::string homeDir = std::string(_homeDataDir + "/zmpPreviewController/");
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, intddhkk).finished(), std::string(homeDir + "refComLinAcc.txt") ,true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentAcceleration).finished(), std::string(homeDir + "currentComLinAcc.txt"),true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, inthkk).finished(), std::string(homeDir + "intComPositionRef.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComPos).finished(), std::string(homeDir + "currentComPos.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, _globalZMP).finished(), std::string(homeDir + "currentZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, pk).finished(), std::string(homeDir + "previewedZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, zmpReference).finished(), std::string(homeDir + "referenceZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, optimalU.topRows(2)).finished(), std::string(homeDir + "optimalInput.txt"), true);

    //TODO: This way of finishing the test is not good. For some reason when the thread is asked to stop, the trajectories go to zero and the robot still tries to track them. Stpping the module with ctrl + c interruption is best.
    if ( el < _zmpTrajectory.size() )
        el++;
    else
        this->askToStop();

}

void WalkingClient::performSingleStepTest() {
    //TODO: Pass most of this shit to the initialization method
    static double timeInit = yarp::os::Time::now();
    static double tnow = yarp::os::Time::now() - timeInit;
    static int el = 0;
    static bool stepStarted = false;

    Eigen::Vector2d zmpReference;
    // Retrieve current COM state
    Eigen::VectorXd hk(6);
    hk.head<2>() = this->model->getCoMPosition().topRows(2);
    hk.segment<2>(2) = this->model->getCoMVelocity().topRows(2);
    hk.tail<2>() = this->model->getCoMAcceleration().topRows(2);
    //TODO: Try to get the task state instead of pos, vel and acc individually.
    //_comTask->getTaskState();
    Eigen::VectorXd hkk(6); hkk.setZero();
    Eigen::Vector2d pk; pk.setZero();
    Eigen::Vector2d intddhkk; intddhkk.setZero();
    Eigen::Vector2d inthkk; inthkk.setZero();
    Eigen::Vector2d intdhkk; intdhkk.setZero();

    zmpReference = _singleStepTrajectory[el];

    // Transform the following Np zmp references from the current time step in the std::vector container to a single Eigen::VectorXd
    transformStdVectorToEigenVector(_singleStepTrajectory, el, _zmpPreviewParams->Np, zmpRefInPreviewWindow);

    // Compute optimal input in preview window for the next Np zmp references
    _zmpPreviewController->computeOptimalInput(zmpRefInPreviewWindow, comVelRefInPreviewWindow, hk, optimalU);

    // Only using the first input computed by the preview controller;
    // This input must now be integrated (since it's just the optimal com jerk)
    _zmpPreviewController->integrateCom(optimalU.topRows(2), hk, hkk);

    // Using the zmp cart model, the instantaneous zmp trajectory can now be
    // computed. For this we'll pass the current full com state hk.
    inthkk = hkk.head<2>();
    intdhkk = hkk.segment<2>(2);
    intddhkk = hkk.tail<2>() + 0.010*optimalU.topRows(2);
    hkk.tail<2>() = intddhkk;
    _zmpPreviewController->tableCartModel(hkk, pk);

    // Apply control
    // Prepare the desired com state and apply control!
    prepareAndsetDesiredCoMTaskState(hkk, true);

    Eigen::Vector3d currentComPos;
    currentComPos = _comTask->getTaskState().getPosition().getTranslation();

    if (tnow > 3 && !stepStarted) {
        Eigen::Vector3d rFootPosition = this->model->getSegmentPosition("r_sole").getTranslation();
        rFootPosition(0) += _singleStepTestParams.stepLength;
        _stepController->deactivateFeetContacts(RIGHT_FOOT);
        _stepController->doStepWithMaxVelocity(RIGHT_FOOT, rFootPosition, _singleStepTestParams.stepHeight);
        stepStarted = true;
    }

    if (stepStarted && _stepController->isTrajectoryFinished(RIGHT_FOOT)) {
        _stepController->activateFeetContacts(RIGHT_FOOT);
    }

    Eigen::Vector3d currentAcceleration;
    currentAcceleration = this->model->getCoMAcceleration();

    // Read Force/Torque measurements
    readFootWrench(LEFT_FOOT, _rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, _rawRightFootWrench);

    // Compute ZMP
    _zmpPreviewController->computeGlobalZMPFromSensors(_rawLeftFootWrench, _rawRightFootWrench, this->model, _globalZMP);

    // Write to file for plotting
    //TODO: Watch out! if the thread doesn't respect the desired period, then your plots will look horizontally scaled!
    tnow = tnow + this->getEstPeriod()/1000;
    std::string homeDir = std::string(_homeDataDir + "/zmpPreviewController/");
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, intddhkk).finished(), std::string(homeDir + "refComLinAcc.txt") ,true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentAcceleration).finished(), std::string(homeDir + "currentComLinAcc.txt"),true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, inthkk).finished(), std::string(homeDir + "intComPositionRef.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComPos).finished(), std::string(homeDir + "currentComPos.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, _globalZMP).finished(), std::string(homeDir + "currentZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, pk).finished(), std::string(homeDir + "previewedZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, zmpReference).finished(), std::string(homeDir + "referenceZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, optimalU.topRows(2)).finished(), std::string(homeDir + "optimalInput.txt"), true);

    //TODO: This way of finishing the test is not good. For some reason when the thread is asked to stop, the trajectories go to zero and the robot still tries to track them. Stpping the module with ctrl + c interruption is best.
    if ( el < _singleStepTrajectory.size() )
        el++;
    else
        this->askToStop();

}

void WalkingClient::steppingTest() {
    static double steppingTestStartTime = yarp::os::Time::now();
    static double initialZmpMoveTime = _steppingTestParams.stepDuration * 1.5;
    static double tnow = yarp::os::Time::now() - steppingTestStartTime;

    static int el = 0;


    Eigen::Vector2d zmpReference;
    // Retrieve current COM state
    Eigen::VectorXd hk(6);
    hk.head<2>() = this->model->getCoMPosition().topRows(2);
    hk.segment<2>(2) = this->model->getCoMVelocity().topRows(2);
    hk.tail<2>() = this->model->getCoMAcceleration().topRows(2);
    //TODO: Try to get the task state instead of pos, vel and acc individually.
    //_comTask->getTaskState();
    Eigen::VectorXd hkk(6); hkk.setZero();
    Eigen::Vector2d pk; pk.setZero();
    Eigen::Vector2d intddhkk; intddhkk.setZero();
    Eigen::Vector2d inthkk; inthkk.setZero();
    Eigen::Vector2d intdhkk; intdhkk.setZero();

    zmpReference = _steppingTrajectory[el];

    // Transform the following Np zmp references from the current time step in the std::vector container to a single Eigen::VectorXd
    transformStdVectorToEigenVector(_steppingTrajectory, el, _zmpPreviewParams->Np, zmpRefInPreviewWindow);

    // Compute optimal input in preview window for the next Np zmp references
    _zmpPreviewController->computeOptimalInput(zmpRefInPreviewWindow, comVelRefInPreviewWindow, hk, optimalU);

    // Only using the first input computed by the preview controller;
    // This input must now be integrated (since it's just the optimal com jerk)
    _zmpPreviewController->integrateCom(optimalU.topRows(2), hk, hkk);

    // Using the zmp cart model, the instantaneous zmp trajectory can now be
    // computed. For this we'll pass the current full com state hk.
    inthkk = hkk.head<2>();
    intdhkk = hkk.segment<2>(2);
    intddhkk = hkk.tail<2>() + 0.010*optimalU.topRows(2);
    hkk.tail<2>() = intddhkk;
    _zmpPreviewController->tableCartModel(hkk, pk);

    // Apply control
    // Prepare the desired com state and apply control!
    prepareAndsetDesiredCoMTaskState(hkk, true);

    Eigen::Vector3d currentComPos;
    currentComPos = _comTask->getTaskState().getPosition().getTranslation();

    if ((yarp::os::Time::now()-steppingTestStartTime)>=initialZmpMoveTime) {
        startSteppinMotherFucker();
    }


    Eigen::Vector3d currentAcceleration;
    currentAcceleration = this->model->getCoMAcceleration();

    // Read Force/Torque measurements
    readFootWrench(LEFT_FOOT, _rawLeftFootWrench);
    readFootWrench(RIGHT_FOOT, _rawRightFootWrench);

    // Compute ZMP
    _zmpPreviewController->computeGlobalZMPFromSensors(_rawLeftFootWrench, _rawRightFootWrench, this->model, _globalZMP);

    // Write to file for plotting
    //TODO: Watch out! if the thread doesn't respect the desired period, then your plots will look horizontally scaled!
    tnow = tnow + this->getEstPeriod()/1000;
    std::string homeDir = std::string(_homeDataDir + "/zmpPreviewController/");
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, intddhkk).finished(), std::string(homeDir + "refComLinAcc.txt") ,true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentAcceleration).finished(), std::string(homeDir + "currentComLinAcc.txt"),true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, inthkk).finished(), std::string(homeDir + "intComPositionRef.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(4) << tnow, currentComPos).finished(), std::string(homeDir + "currentComPos.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, _globalZMP).finished(), std::string(homeDir + "currentZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, pk).finished(), std::string(homeDir + "previewedZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, zmpReference).finished(), std::string(homeDir + "referenceZMP.txt"), true);
    ocra::utils::writeInFile((Eigen::VectorXd(3) << tnow, optimalU.topRows(2)).finished(), std::string(homeDir + "optimalInput.txt"), true);

    //TODO: This way of finishing the test is not good. For some reason when the thread is asked to stop, the trajectories go to zero and the robot still tries to track them. Stpping the module with ctrl + c interruption is best.
    if ( el < _steppingTrajectory.size() )
        ++el;
    else
        this->askToStop();

}

void WalkingClient::startSteppinMotherFucker()
{
    if (!_currentlyStepping) {
        if(_currentStepIndex<_stepOrder.size()){
            std::cout << "Step started." << std::endl;
            auto foot = _stepOrder[_currentStepIndex];
            auto target = _stepTargets.col(_currentStepIndex);
            double stepDuration = _stepTargetDurations(_currentStepIndex);
            _stepController->step(foot, target, stepDuration, _steppingTestParams.stepHeight);
            _currentlyStepping = true;
        }
    } else {
        if (_waitBeforeNextStep) {
            if( ((yarp::os::Time::now() - _waitTimeStart) >= (_steppingTestParams.stepDuration/2.0)) ) {
                if (_stepOrder[_currentStepIndex]==RIGHT_FOOT) {
                    this->changeFixedLink("r_sole", false, true);
                } else {
                    this->changeFixedLink("l_sole", true, false);
                }
                _waitBeforeNextStep = false;
                _currentlyStepping = false;
                ++_currentStepIndex;
            }
        } else {
            if (_stepController->isStepFinished(_stepOrder[_currentStepIndex]) ) {
                std::cout << "Step finished. Waiting for ZMP." << std::endl;
                _waitBeforeNextStep = true;
                _waitTimeStart = yarp::os::Time::now();
            }
        }
    }
}

void WalkingClient::prepareAndsetDesiredCoMTaskState(VectorXd comState, bool doSet)
{
    ocra::TaskState desiredComState;
    Eigen::Vector3d comRefPosition;
    Eigen::Vector3d comRefVelocity;
    Eigen::Vector3d comRefAcceleration;

    if (!_testType.compare("steppingTest")) {
        comRefPosition << comState.head<2>(), _zmpPreviewParams->cz;
        comRefVelocity << comState.segment<2>(2), 0;
    }
    comRefAcceleration << comState.tail<2>(), 0;

    if (doSet) {
        if (!_testType.compare("steppingTest")) {
            desiredComState.setPosition(ocra::util::eigenVectorToDisplacementd(comRefPosition));
            desiredComState.setVelocity(ocra::util::eigenVectorToTwistd(comRefVelocity));
        }
        desiredComState.setAcceleration(ocra::util::eigenVectorToTwistd(comRefAcceleration));
        _comTask->setDesiredTaskStateDirect(desiredComState);
//         _comTask->setDesiredTaskState(desiredComState);
    }

}

std::string WalkingClient::composePortName(std::string portName) {
    std::string tmp;
    tmp = std::string("/" + _clientName + "/" + portName);
    return tmp;
}

void WalkingClient::findZMPPreviewControllerParams(yarp::os::ResourceFinder &rf) {
        if (!rf.check("ZMP_PREVIEW_CONTROLLER_PARAMS")) {
        OCRA_WARNING("Group of options ZMP_PREVIEW_CONTROLLER_PARAMS was not found, using default parameters Nc=3, nu=, nw=, nb= ");
    } else {
        yarp::os::Property zmpPreviewControllerParamsGroup;
        zmpPreviewControllerParamsGroup.fromString(rf.findGroup("ZMP_PREVIEW_CONTROLLER_PARAMS").tail().toString());
        _zmpPreviewParams->cz = this->model->getCoMPosition().operator()(2);
        _zmpPreviewParams->Nc = zmpPreviewControllerParamsGroup.find("Nc").asInt();
        _zmpPreviewParams->Np = zmpPreviewControllerParamsGroup.find("Np").asInt();
        _zmpPreviewParams->nw = zmpPreviewControllerParamsGroup.find("nw").asDouble();
        _zmpPreviewParams->nb = zmpPreviewControllerParamsGroup.find("nb").asDouble();
        _zmpPreviewParams->nu = zmpPreviewControllerParamsGroup.find("nu").asDouble();
        OCRA_INFO(">> [ZMP_PREVIEW_CONTROLLER_PARAMS]: \n " << zmpPreviewControllerParamsGroup.toString().c_str() << " cz: " << _zmpPreviewParams->cz);
    }

    // Create zmpPreviewController object
   _zmpPreviewController = std::make_shared<ZmpPreviewController>((double) this->getExpectedPeriod(), _zmpPreviewParams);
}

void WalkingClient::findGeneralTestsParams(yarp::os::ResourceFinder &rf) {
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
}

void WalkingClient::findCOMLinVelConstRefParams(yarp::os::ResourceFinder &rf) {
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
}

void WalkingClient::findZMPConstRefParams(yarp::os::ResourceFinder &rf) {
    if (!rf.check("ZMP_CONSTANT_REFERENCE")) {
    OCRA_WARNING("Group ZMP_CONSTANT_REFERENCE was not found, using default parameters zmpYConstRef=-0.10, stopTimeConstZmp=3");
    _zmpYConstRef = -0.010;
    _stopTimeConstZmp = 3;
    } else {
        yarp::os::Property zmpConstantReferenceGroup;
        zmpConstantReferenceGroup.fromString(rf.findGroup("ZMP_CONSTANT_REFERENCE").tail().toString());
        _zmpYConstRef = zmpConstantReferenceGroup.find("zmpYConstRef").asDouble();
        _stopTimeConstZmp = zmpConstantReferenceGroup.find("stopTimeConstZmp").asDouble();
        _riseTimeConstZmp = zmpConstantReferenceGroup.find("riseTime").asDouble();
        _trajectoryDurationConstZmp = zmpConstantReferenceGroup.find("duration").asDouble();;
        OCRA_INFO(">> [ZMP_CONSTANT_REFERENCE]: \n " << zmpConstantReferenceGroup.toString().c_str());
    }
}

void WalkingClient::findSingleStepTestParams(yarp::os::ResourceFinder &rf) {
        if (!rf.check("SINGLE_STEP_TEST")) {
        OCRA_WARNING("Group SINGLE_STEP_TEST was not found, using default parameters");
    } else {
        yarp::os::Property singleStepTestGroup;
        singleStepTestGroup.fromString(rf.findGroup("SINGLE_STEP_TEST").tail().toString());
        _singleStepTestParams.totalDuration = singleStepTestGroup.find("totalDuration").asDouble();
        _singleStepTestParams.offset = singleStepTestGroup.find("offset").asDouble();
        _singleStepTestParams.SSduration = singleStepTestGroup.find("SSduration").asDouble();
        _singleStepTestParams.riseTime = singleStepTestGroup.find("riseTime").asDouble();
        _singleStepTestParams.stepLength = singleStepTestGroup.find("stepLength").asDouble();
        _singleStepTestParams.stepHeight = singleStepTestGroup.find("stepHeight").asDouble();
        OCRA_INFO(">> [SINGLE_STEP_TEST]: \n " << singleStepTestGroup.toString().c_str());
    }
}

void WalkingClient::findZMPVaryingReferenceParams(yarp::os::ResourceFinder &rf) {
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
}

void WalkingClient::findMIQPParams(yarp::os::ResourceFinder &rf) {
    if (!rf.check("MIQP_CONTROLLER_PARAMS")) {
        OCRA_ERROR("No parameters have been specified for the MIQP controller");
    } else {
        OCRA_ERROR("rf " << rf.toString());
        yarp::os::Property miqpParamsGroup;
        OCRA_ERROR("rf to string: "<< rf.findGroup("MIQP_CONTROLLER_PARAMS").tail().toString());
        miqpParamsGroup.fromString(rf.findGroup("MIQP_CONTROLLER_PARAMS").tail().toString());
        OCRA_ERROR("rf group params from string: " << miqpParamsGroup.toString());
        _miqpParams.cz = this->model->getCoMPosition().operator()(2);
        _miqpParams.dCoMxRef = miqpParamsGroup.find("dCoMxRef").asDouble();
        _miqpParams.dCoMyRef = miqpParamsGroup.find("dCoMyRef").asDouble();
        _miqpParams.dt = (unsigned int) miqpParamsGroup.find("dt").asInt();
        _miqpParams.dtThread = (unsigned int) miqpParamsGroup.find("dtThread").asInt();
        _miqpParams.g = miqpParamsGroup.find("g").asDouble();
        _miqpParams.home = _homeDataDir; // This is actually in the TESTS_GENERAL_PARAMETERS group
        _miqpParams.N = miqpParamsGroup.find("N").asInt();
        _miqpParams.sx_constancy = miqpParamsGroup.find("sx_constancy").asDouble();
        _miqpParams.sy_constancy = miqpParamsGroup.find("sy_constancy").asDouble();
        _miqpParams.sx_ss = miqpParamsGroup.find("sx_ss").asDouble();
        _miqpParams.sy_ss = miqpParamsGroup.find("sy_ss").asDouble();
        _miqpParams.hx_ref = miqpParamsGroup.find("hx_ref").asDouble();
        _miqpParams.hy_ref = miqpParamsGroup.find("hy_ref").asDouble();
        _miqpParams.dhx_ref = miqpParamsGroup.find("dhx_ref").asDouble();
        _miqpParams.dhy_ref = miqpParamsGroup.find("dhy_ref").asDouble();
        _miqpParams.ddhx_ref = miqpParamsGroup.find("ddhx_ref").asDouble();
        _miqpParams.ddhy_ref =miqpParamsGroup.find("ddhy_ref").asDouble();
        _miqpParams.ww = miqpParamsGroup.find("ww").asDouble();
        _miqpParams.wb = miqpParamsGroup.find("wb").asDouble();
        _miqpParams.wu = miqpParamsGroup.find("wu").asDouble();
        _miqpParams.wss = miqpParamsGroup.find("wss").asDouble();
        _miqpParams.wstep = miqpParamsGroup.find("wstep").asDouble();
        _miqpParams.wdelta = miqpParamsGroup.find("wdelta").asDouble();
        _miqpParams.shapeConstraints = miqpParamsGroup.find("shapeConstraints").asBool();
        _miqpParams.admissibilityConstraints = miqpParamsGroup.find("admissibilityConstraints").asBool();
        _miqpParams.copConstraints = miqpParamsGroup.find("copConstraints").asBool();
        _miqpParams.walkingConstraints = miqpParamsGroup.find("walkingConstraints").asBool();
        _miqpParams.addRegularization = miqpParamsGroup.find("addRegularization").asBool();
        _miqpParams.robot = miqpParamsGroup.find("robot").asString();
        _miqpParams.marginCoPBounds = miqpParamsGroup.find("marginCoPBounds").asDouble();
         OCRA_INFO(">> [MIQP_CONTROLLER_PARAMS in config file]: \n " << miqpParamsGroup.toString().c_str());
    }
}

void WalkingClient::findSteppingTestParams(yarp::os::ResourceFinder &rf) {
    if (!rf.check("STEPPING_TEST")) {
        OCRA_WARNING("Group STEPPING_TEST was not found, using default parameters");
    } else {
        yarp::os::Property steppingTestGroup;
        steppingTestGroup.fromString(rf.findGroup("STEPPING_TEST").tail().toString());
        _steppingTestParams.nSteps = steppingTestGroup.find("nSteps").asInt();
        _steppingTestParams.stepDuration = steppingTestGroup.find("stepDuration").asDouble();
        _steppingTestParams.stepLength = steppingTestGroup.find("stepLength").asDouble();
        _steppingTestParams.stepHeight = steppingTestGroup.find("stepHeight").asDouble();
        OCRA_INFO(">> [STEPPING_TEST]: \n " << steppingTestGroup.toString().c_str());
    }
}
