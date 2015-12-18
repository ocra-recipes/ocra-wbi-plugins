#include <taskSequences/sequences/Exploration.h>
#include <ocraWbiPlugins/ocraWbiModel.h>

#ifndef ERROR_THRESH
#define ERROR_THRESH 0.03 // Goal error threshold for hand tasks
#endif

#ifndef VAR_THRESH
#define VAR_THRESH 0.99
#endif

#ifndef TIME_LIMIT
#define TIME_LIMIT 15.0 // Maximum time to be spent on any trajectory.
#endif

Exploration::~Exploration()
{
    l_hand_port.close();
    l_hand_target_port.close();
    r_hand_port.close();
    r_hand_target_port.close();
}


void Exploration::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    varianceThresh = Eigen::Array3d::Constant(VAR_THRESH);

    /*
    *   Task coefficients
    */
    bool usesYARP = true;
    //  fullPosture
    double Kp_fullPosture       = 5.0;
    double Kd_fullPosture       = 2.0 * sqrt(Kp_fullPosture);
    double weight_fullPosture   = 0.0001;

    //  torsoPosture
    double Kp_torsoPosture      = 5.0;
    double Kd_torsoPosture      = 2.0 * sqrt(Kp_torsoPosture);
    double weight_torsoPosture  = 0.01;

    //  leftHand
    double Kp_leftHand = 10.0;
    double Kd_leftHand = 2.0 *sqrt(Kp_leftHand);
    Eigen::Vector3d weights_leftHand = Eigen::Vector3d::Ones(3);

    //  rightHand
    double Kp_rightHand = 10.0;
    double Kd_rightHand = 2.0 *sqrt(Kp_rightHand);
    Eigen::Vector3d weights_rightHand = Eigen::Vector3d::Ones(3);


    /*
    *   Task constructors
    */

    // fullPosture
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getHomePosture(model, nominal_q);

    taskManagers["fullPosture"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPosture", ocra::FullState::INTERNAL, Kp_fullPosture, Kd_fullPosture, weight_fullPosture, nominal_q, usesYARP);


    // torsoPosture
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << 0.0, 0.0, 0.0;

    taskManagers["torsoPosture"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "torsoPosture", ocra::FullState::INTERNAL, torso_indices, Kp_torsoPosture, Kd_torsoPosture, weight_torsoPosture, torsoTaskPosDes, usesYARP);


    //  leftHand
    Eigen::Vector3d l_handDisp(0.05, 0.0, 0.0); // Moves the task frame to the center of the hand.
    taskManagers["leftHand"] = new wocra::wOcraVariableWeightsTaskManager(ctrl, model, "leftHand", "l_hand", l_handDisp, Kp_leftHand, Kd_leftHand, weights_leftHand, usesYARP);

    //  rightHand
    Eigen::Vector3d r_handDisp(0.05, 0.0, 0.0); // Moves the task frame to the center of the hand.
    taskManagers["rightHand"] = new wocra::wOcraVariableWeightsTaskManager(ctrl, model, "rightHand", "r_hand", r_handDisp, Kp_rightHand, Kd_rightHand, weights_rightHand, usesYARP);


    /*
    *   Trajectory constructors
    */

    leftHandTrajectory = new wocra::wOcraExperimentalTrajectory();
    rightHandTrajectory = new wocra::wOcraExperimentalTrajectory();

    // leftHandTrajectory->setWaypoints(startingPos, desiredPos)

    /*
    *   Cast tasks to derived classes to access their virtual functions
    */
    leftHandTask = dynamic_cast<wocra::wOcraVariableWeightsTaskManager*>(taskManagers["leftHand"]);

    rightHandTask = dynamic_cast<wocra::wOcraVariableWeightsTaskManager*>(taskManagers["rightHand"]);


    /*
    *   Variables used in the doUpdate control logic
    */
    lHandIndex = model.getSegmentIndex("l_hand");
    rHandIndex = model.getSegmentIndex("r_hand");


    initTrigger = true;

    std::string l_hand_port_name = "/lHandFrame:o";
    l_hand_port.open(l_hand_port_name.c_str());
    yarp.connect(l_hand_port_name.c_str(), "/leftHandSphere:i");

    std::string l_hand_target_port_name = "/lHandTarget:o";
    l_hand_target_port.open(l_hand_target_port_name.c_str());
    yarp.connect(l_hand_target_port_name.c_str(), "/leftHandTargetSphere:i");

    std::string r_hand_port_name = "/rHandFrame:o";
    r_hand_port.open(r_hand_port_name.c_str());
    yarp.connect(r_hand_port_name.c_str(), "/rightHandSphere:i");

    std::string r_hand_target_port_name = "/rHandTarget:o";
    r_hand_target_port.open(r_hand_target_port_name.c_str());
    yarp.connect(r_hand_target_port_name.c_str(), "/rightHandTargetSphere:i");
}



void Exploration::doUpdate(double time, wocra::wOcraModel& state, void** args)
{

    Eigen::Vector3d currentLeftHandPos = (leftHandTask->getTaskFramePosition() + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1, -1, 1);
    Eigen::Vector3d currentRightHandPos = (rightHandTask->getTaskFramePosition() + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1, -1, 1);


    yarp::os::Bottle l_hand_output, r_hand_output;
    for(int i=0; i<3; i++){
        l_hand_output.addDouble(currentLeftHandPos(i));
        r_hand_output.addDouble(currentRightHandPos(i));
    }
    l_hand_port.write(l_hand_output);
    r_hand_port.write(r_hand_output);


    if (initTrigger) {
        resetTimeLeft = time;
        resetTimeRight = time;
        generateNewWaypoints(state, lHandIndex);
        initTrigger = false;
        generateNewWaypoints(state, rHandIndex);
        std::cout << "Generating new left hand target @ " << currentDesiredPosition_leftHand.transpose() << std::endl;
        std::cout << "Generating new right hand target @ " << currentDesiredPosition_rightHand.transpose() << std::endl;


    }


    if ( (std::abs(time - resetTimeLeft) >= TIME_LIMIT) || (attainedGoal(state, lHandIndex)) )
    {
        generateNewWaypoints(state, lHandIndex);
        resetTimeLeft = time;
    }
    else
    {

        leftHandTrajectory->getDesiredValues(time, desiredPosVelAcc_leftHand, desiredVariance_leftHand);
        desiredWeights_leftHand = mapVarianceToWeights(desiredVariance_leftHand);

        leftHandTask->setState(desiredPosVelAcc_leftHand.col(0));
        leftHandTask->setWeights(desiredWeights_leftHand);

    }

    if ( (std::abs(time - resetTimeRight) >= TIME_LIMIT) || (attainedGoal(state, rHandIndex)) )
    {
        generateNewWaypoints(state, rHandIndex);
        resetTimeRight = time;

    }
    else
    {
        rightHandTrajectory->getDesiredValues(time, desiredPosVelAcc_rightHand, desiredVariance_rightHand);
        desiredWeights_rightHand = mapVarianceToWeights(desiredVariance_rightHand);

        rightHandTask->setState(desiredPosVelAcc_rightHand.col(0));
        rightHandTask->setWeights(desiredWeights_rightHand);

    }

    Eigen::Vector3d currentDesiredPosition_leftHand_transformed = (currentDesiredPosition_leftHand + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);
    Eigen::Vector3d currentDesiredPosition_rightHand_transformed = (currentDesiredPosition_rightHand + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);
    yarp::os::Bottle l_hand_target_output, r_hand_target_output;
    for(int i=0; i<3; i++){
        l_hand_target_output.addDouble(currentDesiredPosition_leftHand_transformed(i));
        r_hand_target_output.addDouble(currentDesiredPosition_rightHand_transformed(i));
    }
    l_hand_target_port.write(l_hand_target_output);
    r_hand_target_port.write(r_hand_target_output);


    // if(isCartesion){std::cout << "\nFinal desired position: " << desiredPos.transpose() << std::endl;}
    // std::cout << "\nDesired position: " << desiredPosVelAcc.col(0).transpose() << std::endl;
    // std::cout << "Current position: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
    // std::cout << "Error: " << tmp_tmLeftHandCart->getTaskError().transpose() << "   norm: " << tmp_tmLeftHandCart->getTaskErrorNorm() << std::endl;




}

bool Exploration::attainedGoal(wocra::wOcraModel& state, int segmentIndex)
{
    double error;
    Eigen::Vector3d currentDesiredPosition, taskFrame;
    if (segmentIndex==lHandIndex) {
        currentDesiredPosition = currentDesiredPosition_leftHand;
        taskFrame = leftHandTask->getTaskFramePosition();
    }
    else if (segmentIndex==rHandIndex) {
        currentDesiredPosition = currentDesiredPosition_rightHand;
        taskFrame = rightHandTask->getTaskFramePosition();

    }
    else{
        std::cout << "[ERROR] Exploration::attainedGoal - segment name doesn't match either l_hand or r_hand" << std::endl;
        return false;
    }

    error = (currentDesiredPosition - taskFrame ).norm();
    bool result = error <= ERROR_THRESH;
    return result;
}


Eigen::VectorXd Exploration::mapVarianceToWeights(Eigen::VectorXd& variance)
{
    double beta = 1.0;
    variance /= maxVariance;
    variance = variance.array().min(varianceThresh); //limit variance to 0.99 maximum
    Eigen::VectorXd weights = (Eigen::VectorXd::Ones(variance.rows()) - variance) / beta;
    return weights;
}


void Exploration::generateNewWaypoints(wocra::wOcraModel& state, int segmentIndex)
{
    std::cout << "\n==================" << std::endl;
    Eigen::VectorXd startPoint = state.getSegmentPosition(segmentIndex).getTranslation();
    if (segmentIndex==lHandIndex) {
        currentDesiredPosition_leftHand = generateTarget(lHandIndex);
        std::cout << "Generating new left hand target @ " << currentDesiredPosition_leftHand.transpose() << std::endl;
        leftHandTrajectory->setWaypoints(startPoint, currentDesiredPosition_leftHand);
        // std::cout << "Waypoints set for left hand" << std::endl;

    }
    else if (segmentIndex==rHandIndex) {
        currentDesiredPosition_rightHand = generateTarget(rHandIndex);
        std::cout << "Generating new right hand target @ " << currentDesiredPosition_rightHand.transpose() << std::endl;
        rightHandTrajectory->setWaypoints(startPoint, currentDesiredPosition_rightHand);
        // std::cout << "Waypoints set for right hand" << std::endl;

    }
    else{std::cout << "Houston we have a problem..." << std::endl;}


    if (!initTrigger) {
        maxVariance = leftHandTrajectory->getMaxVariance() > rightHandTrajectory->getMaxVariance() ? leftHandTrajectory->getMaxVariance() : rightHandTrajectory->getMaxVariance();
        std::cout << "Maximum variance is: " << maxVariance << std::endl;
    }
    std::cout << "==================\n" << std::endl;
}

Eigen::VectorXd Exploration::generateTarget(int segmentIndex)
{
    Eigen::Vector3d newTarget;

    if (segmentIndex==lHandIndex)
    {
        double x_min_l = -0.4;      double x_max_l = 0.1;//-0.15;
        double y_min_l = -0.2;      double y_max_l = 0.2;
        double z_min_l = -0.3;      double z_max_l = 0.5;

        double xRange_l = x_max_l - x_min_l;
        double yRange_l = y_max_l - y_min_l;
        double zRange_l = z_max_l - z_min_l;

        // newTarget(0) = x_min_l + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(xRange_l)));
        // newTarget(1) = y_min_l + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(yRange_l)));
        // newTarget(2) = z_min_l + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(zRange_l)));

        Eigen::Vector3d range_l(xRange_l, yRange_l, zRange_l);
        Eigen::Vector3d min_l(x_min_l, y_min_l, z_min_l);

        newTarget = min_l + (Eigen::Vector3d::Random().array().abs().array() * range_l.array()).matrix();
    }
    else if(segmentIndex == rHandIndex)
    {
        double x_min_r = -0.4;      double x_max_r = 0.1;//-0.15;
        double y_min_r = -0.2;     double y_max_r = 0.2;
        double z_min_r = -0.3;      double z_max_r = 0.5;

        double xRange_r = x_max_r - x_min_r;
        double yRange_r = y_max_r - y_min_r;
        double zRange_r = z_max_r - z_min_r;

        // newTarget(0) = x_min_r + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(xRange_r)));
        // newTarget(1) = y_min_r + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(yRange_r)));
        // newTarget(2) = z_min_r + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(zRange_r)));

        Eigen::Vector3d range_r(xRange_r, yRange_r, zRange_r);
        Eigen::Vector3d min_r(x_min_r, y_min_r, z_min_r);

        newTarget = min_r + (Eigen::Vector3d::Random().array().abs().array() * range_r.array()).matrix();
    }

    return newTarget;

}
