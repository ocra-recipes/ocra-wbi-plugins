#include <taskSequences/sequences/TaskOptimization.h>

#ifndef ERROR_THRESH
#define ERROR_THRESH 0.03 // Goal error threshold for hand tasks
#endif

#ifndef VAR_THRESH
#define VAR_THRESH 0.99
#endif

#ifndef TIME_LIMIT
#define TIME_LIMIT 10.0 // Maximum time to be spent on any trajectory.
#endif

TaskOptimization::TaskOptimization()
{

    useVarianceModulation = true;

    runObstacleTest_1D = true;
    runObstacleTest_3D = false;
    runArmCrossingTest = false;

    // Must come after the runObstacleTest variables...
    connectYarpPorts();

    if (runObstacleTest_1D) {
        obstacleTime = 1.0;
        bOptCovarianceScalingFactor = 6.0;

    }else if (runObstacleTest_3D) {
        obstacleTime = 0.0;
        bOptCovarianceScalingFactor = 5.0;
    }else if (runArmCrossingTest) {
        bOptCovarianceScalingFactor = 7.0;
    }

    replayOptimalTrajectory = true;

    useGoalCost = true;

    useTrackingCost = true;

    useEnergyCost = true;

    logTrajectoryData = true;

    rootLogFilePathPrefix = "/home/ryan/Dropbox/RSS_2016/raw_data/";//"/home/ryan/Desktop/tmp-test/";

    if (runObstacleTest_1D) {
        rootLogFilePathPrefix += "ObstacleTest_1D";
    }else if (runObstacleTest_3D) {
        rootLogFilePathPrefix += "ObstacleTest_3D";
    }else if (runArmCrossingTest) {
        rootLogFilePathPrefix += "ArmCrossingTest";
    }

    if (useVarianceModulation)
    {
        rootLogFilePathPrefix += "_VarianceModulated";
    }

    smlt::checkAndCreateDirectory(rootLogFilePathPrefix);
    std::ofstream pathFile;
    pathFile.open((rootLogFilePathPrefix+"/latestLogPath.txt").c_str());

    rootLogFilePathPrefix += "/test-" + smlt::currentDateTime() +"/";
    smlt::checkAndCreateDirectory(rootLogFilePathPrefix);

    pathFile << rootLogFilePathPrefix;
    pathFile.close();

}

TaskOptimization::~TaskOptimization()
{
    optVarsPortOut.close();
    costPortOut.close();
    optVarsPortIn.close();
    optParamsPortOut.close();

    r_hand_port.close();
    r_hand_target_port.close();
    r_hand_start_port.close();
    r_hand_waypoint_port.close();

    obstacle_port.close();

}

bool TaskOptimization::openLogFiles(const std::string testLogFilePathPrefix)
{
    bool retVal = true;
    if (logTrajectoryData)
    {
        realTrajectoryFile.open((testLogFilePathPrefix + "/realTrajectory.txt").c_str());
        retVal = retVal && realTrajectoryFile.is_open();


        totalInstantaneousCostFile.open((testLogFilePathPrefix + "/totalInstantaneousCost.txt").c_str());
        retVal = retVal && totalInstantaneousCostFile.is_open();

        if(useGoalCost)
        {
            goalInstantaneousCostFile.open((testLogFilePathPrefix + "/goalInstantaneousCost.txt").c_str());
            retVal = retVal && goalInstantaneousCostFile.is_open();
        }
        if(useTrackingCost)
        {
            trackingInstantaneousCostFile.open((testLogFilePathPrefix + "/trackingInstantaneousCost.txt").c_str());
            retVal = retVal && trackingInstantaneousCostFile.is_open();
        }
        if(useEnergyCost)
        {
            energyInstantaneousCostFile.open((testLogFilePathPrefix + "/energyInstantaneousCost.txt").c_str());
            retVal = retVal && energyInstantaneousCostFile.is_open();
        }
    }
    return retVal;

}

bool TaskOptimization::closeLogFiles()
{
    bool retVal = true;
    if (logTrajectoryData)
    {
        realTrajectoryFile.close();
        retVal = retVal && !realTrajectoryFile.is_open();

        totalInstantaneousCostFile.close();
        retVal = retVal && !totalInstantaneousCostFile.is_open();


        if(useGoalCost)
        {
            goalInstantaneousCostFile.close();
            retVal = retVal && !goalInstantaneousCostFile.is_open();

        }

        if(useTrackingCost)
        {
            trackingInstantaneousCostFile.close();
            retVal = retVal && !trackingInstantaneousCostFile.is_open();

        }

        if(useEnergyCost)
        {
            energyInstantaneousCostFile.close();
            retVal = retVal && !energyInstantaneousCostFile.is_open();

        }

    }
    return retVal;
}

void TaskOptimization::connectYarpPorts()
{
    optVarsPortOut_name = "/opt/task/vars:o";
    costPortOut_name = "/opt/task/cost:o";
    optVarsPortIn_name = "/opt/task/vars:i";
    optParamsPortOut_name = "/opt/task/params:o";

    optVarsPortOut.open(optVarsPortOut_name.c_str());
    costPortOut.open(costPortOut_name.c_str());
    optVarsPortIn.open(optVarsPortIn_name.c_str());
    optParamsPortOut.open(optParamsPortOut_name.c_str());

    double connectionElapsedTime = 0.0;
    double waitInterval = 2.0;
    double connectionTimeOut = 20.0;

    while (!yarp.connect(optVarsPortOut_name.c_str(), "/opt/solver/vars:i") && connectionElapsedTime <= connectionTimeOut ){
        std::cout << "Waiting to connect to solver ports. Please make sure the taskOptimizer module is running." << std::endl;
        yarp::os::Time::delay(waitInterval);
        connectionElapsedTime += waitInterval;
    }
    connectionElapsedTime = 0.0;
    while (!yarp.connect(costPortOut_name.c_str(), "/opt/solver/cost:i") && connectionElapsedTime <= connectionTimeOut ){
        std::cout << "Waiting to connect to solver ports. Please make sure the taskOptimizer module is running." << std::endl;
        yarp::os::Time::delay(waitInterval);
        connectionElapsedTime += waitInterval;
    }
    connectionElapsedTime = 0.0;
    while (!yarp.connect("/opt/solver/vars:o", optVarsPortIn_name.c_str()) && connectionElapsedTime <= connectionTimeOut ){
        std::cout << "Waiting to connect to solver ports. Please make sure the taskOptimizer module is running." << std::endl;
        yarp::os::Time::delay(waitInterval);
        connectionElapsedTime += waitInterval;
    }

    connectionElapsedTime = 0.0;
    while (!yarp.connect(optParamsPortOut_name.c_str(), "/opt/solver/params:i") && connectionElapsedTime <= connectionTimeOut ){
        std::cout << "Waiting to connect to solver ports. Please make sure the taskOptimizer module is running." << std::endl;
        yarp::os::Time::delay(waitInterval);
        connectionElapsedTime += waitInterval;
    }

    std::string r_hand_port_name = "/rHandFrame:o";
    r_hand_port.open(r_hand_port_name.c_str());
    yarp.connect(r_hand_port_name.c_str(), "/rightHandSphere:i");

    std::string r_hand_start_port_name = "/rHandStart:o";
    r_hand_start_port.open(r_hand_start_port_name.c_str());
    yarp.connect(r_hand_start_port_name.c_str(), "/startSphere:i");

    std::string r_hand_target_port_name = "/rHandTarget:o";
    r_hand_target_port.open(r_hand_target_port_name.c_str());
    yarp.connect(r_hand_target_port_name.c_str(), "/rightHandTargetSphere:i");

    std::string r_hand_waypoint_port_name = "/rHandWaypoint:o";
    r_hand_waypoint_port.open(r_hand_waypoint_port_name.c_str());
    yarp.connect(r_hand_waypoint_port_name.c_str(), "/rightHandWaypointSphere:i");

    std::string obstacle_port_name = "/obstacle:o";
    obstacle_port.open(obstacle_port_name.c_str());

    if (runObstacleTest_1D) {
        yarp.connect(obstacle_port_name.c_str(), "/thinPlate:i");
    }
    else if (runObstacleTest_3D) {
        yarp.connect(obstacle_port_name.c_str(), "/boxObstacle:i");
    }
}

void TaskOptimization::doInit(ocra::Controller& ctrl, ocra::Model& model)
{



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
    double Kp_torsoPosture      = 30.0;
    double Kd_torsoPosture      = 2.0 * sqrt(Kp_torsoPosture);
    // double weight_torsoPosture  = 0.001;
    double weight_torsoPosture  = 0.01;


    //  rightHand
    double Kp_rightHand = 20.0;
    double Kd_rightHand = 2.0 *sqrt(Kp_rightHand);
    Eigen::Vector3d weights_rightHand = Eigen::Vector3d::Ones(3);


    /*
    *   Task constructors
    */

    // fullPosture
    std::cout << "model.nbInternalDofs(): " << model.nbInternalDofs() << std::endl;

    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getHomePosture(model, nominal_q);

    taskManagers["fullPosture"] = std::make_shared<ocra::FullPostureTaskManager>(ctrl, model, "fullPosture", ocra::FullState::INTERNAL, Kp_fullPosture, Kd_fullPosture, weight_fullPosture, nominal_q, usesYARP);


    // torsoPosture
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");
    torsoTaskPosDes << 0.0, 0.0, 0.0;

    taskManagers["torsoPosture"] = std::make_shared<ocra::PartialPostureTaskManager>(ctrl, model, "torsoPosture", ocra::FullState::INTERNAL, torso_indices, Kp_torsoPosture, Kd_torsoPosture, weight_torsoPosture, torsoTaskPosDes, usesYARP);


    //  rightHand
    Eigen::Vector3d r_handDisp(0.05, 0.0, 0.0); // Moves the task frame to the center of the hand.
    taskManagers["rightHand"] = std::make_shared<ocra::VariableWeightsTaskManager>(ctrl, model, "rightHand", "r_hand", r_handDisp, Kp_rightHand, Kd_rightHand, weights_rightHand, usesYARP);


    /*
    *   Trajectory constructors
    */

    rightHandTrajectory = new ocra::GaussianProcessTrajectory();


    /*
    *   Cast tasks to derived classes to access their virtual functions
    */
    rightHandTask = dynamic_cast<ocra::VariableWeightsTaskManager*>(taskManagers["rightHand"].get());

    if (runObstacleTest_1D)
    {
        /*
        *   Starting Waypoint
        */
        rHandPosStart(0) = -0.35; // X
        rHandPosStart(1) = 0.2; // Y
        rHandPosStart(2) = 0.0; // Z

        /*
        *   Ending Waypoint
        */
        dofIndex = 2;
        Eigen::Vector3d rHandDisplacement = Eigen::Vector3d::Zero();
        rHandDisplacement(dofIndex) = 0.25; // meters
        rHandPosEnd = rHandPosStart + rHandDisplacement;
    }
    else if (runObstacleTest_3D)
    {
        /*
        *   Starting Waypoint
        */
        rHandPosStart << -0.2, 0.3, 0.0;

        /*
        *   Ending Waypoint
        */
        // dofIndex = 2;
        Eigen::Vector3d rHandDisplacement;
        rHandDisplacement << -0.2, -0.2, 0.2;
        rHandPosEnd = rHandPosStart + rHandDisplacement;
    }

    /*
    *   Set waypoints and traj
    */
    rightHandTrajectory->setWaypoints(rHandPosStart, rHandPosEnd);




    if (runObstacleTest_1D)
    {
        /*
        *   Set opt variables
        */
        std::vector<Eigen::VectorXi> dofToOptimize(1);
        dofToOptimize[0].resize(2);
        dofToOptimize[0] << 0, dofIndex+1;
        optVariables = rightHandTrajectory->getBoptVariables(1, dofToOptimize);
        std::vector<bool> varBoolVec;
        varBoolVec.push_back(true);
        varBoolVec.push_back(true);
        varBoolVec.push_back(false);
        rightHandTrajectory->setVarianceWaypoints(varBoolVec);
        rightHandTrajectory->printWaypointData();

    }

    else if (runObstacleTest_3D)
    {
        /*
        *   Set opt variables
        */
        std::vector<Eigen::VectorXi> dofToOptimize(1);
        dofToOptimize[0].resize(3);
        dofToOptimize[0] << 1,2,3;
        optVariables = rightHandTrajectory->getBoptVariables(1, dofToOptimize);

        // optVariables = rightHandTrajectory->getBoptVariables(1);
    }

    /*
    *   Send opt params
    */
    sendOptimizationParameters();



    /*
    *   Get max variance for weight calcs.
    */
    maxVariance = rightHandTrajectory->getMaxVariance();


    /*
    *   Variables used in the doUpdate control logic
    */
    rHandIndex = model.getSegmentIndex("r_hand");
    waitForSolver = false;
    waitForHomePosition = false;
    optimumFound = false;
    sequenceFinished = false;
    initTrigger = true;
    newOptVarsReceived = false;
    dataSent_AwaitReply = false;
    testNumber = 1;

    /*
    *   For the trajectory logging
    */
    desiredPosVelAcc_rightHand = Eigen::MatrixXd::Zero(3,3);
    desiredPosVelAcc_rightHand.col(0) << rHandPosStart;


    /*
    *   For gazebo
    */
    rightHandGoalPosition = rHandPosEnd;
    rightHandGoalPosition_transformed = (rightHandGoalPosition + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);

    rightHandStartPosition_transformed = (rHandPosStart + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);


    if (runObstacleTest_1D) {
        currentOptWaypoint = rHandPosEnd;
        currentOptWaypoint(dofIndex) = optVariables(1);
    }else if (runObstacleTest_3D) {
        currentOptWaypoint = optVariables.tail(3);
        obstacle3DGazeboPosition = (currentOptWaypoint + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);

    }

    /*
    *   Informative
    */
    std::cout << "===============================================================\n \t Test Parameters\n===============================================================\n" << std::endl;
    std::cout << "rHandPosStart = "<<rHandPosStart.transpose() << "\nrHandPosEnd = " << rHandPosEnd.transpose() << std::endl;
    std::cout << "optVariables:" << optVariables.transpose() << std::endl;

    std::cout << "\n===============================================================\n" << std::endl;

}

void TaskOptimization::sendOptimizationParameters()
{
    yarp::os::Bottle& optParamsBottle = optParamsPortOut.prepare();
    optParamsBottle.clear();
    int nDims = optVariables.size();
    optParamsBottle.addInt(nDims);
    Eigen::VectorXd searchSpaceMin = rightHandTrajectory->getBoptSearchSpaceMinBound();
    Eigen::VectorXd searchSpaceMax = rightHandTrajectory->getBoptSearchSpaceMaxBound();

    for(int i=0; i<nDims; i++)
    {
        optParamsBottle.addDouble(searchSpaceMin(i));
    }

    for(int i=0; i<nDims; i++)
    {
        optParamsBottle.addDouble(searchSpaceMax(i));
    }
    optParamsBottle.addString(rootLogFilePathPrefix.c_str());
    optParamsBottle.addString("Solver");

    optParamsBottle.addDouble(bOptCovarianceScalingFactor);


    optParamsPortOut.write();
}

void TaskOptimization::initializeTrajectory(double time)
{
    resetTimeRight = time;
    initTrigger = false;
    costIterCounter = 0;
    int nRowsMax = ceil((TIME_LIMIT / 0.01)*2);
    if (useGoalCost){
        goalCostMat.resize(nRowsMax, 2);
    }
    if (useTrackingCost){
        trackingCostMat.resize(nRowsMax, 2);
    }
    if (useEnergyCost){
        energyCostMat.resize(nRowsMax, 2);
    }


    if (logTrajectoryData) {
        // Convert an int to a string pre C++11 safe
        std::ostringstream intAsStream; intAsStream << testNumber;
        std::string testLogFilePathPrefix = intAsStream.str();
        // Append log file path with test number
        testLogFilePathPrefix = rootLogFilePathPrefix +"/Trajectory/"+ testLogFilePathPrefix;
        smlt::checkAndCreateDirectory(testLogFilePathPrefix);
        rightHandTrajectory->saveTrajectoryToFile(testLogFilePathPrefix);
        rightHandTrajectory->saveWaypointDataToFile(testLogFilePathPrefix);
        if(!openLogFiles(testLogFilePathPrefix))
        {
            std::cout << "[ERROR](line: "<< __LINE__ <<") -> Could not open data log files for the trajectory! Tried: "<< testLogFilePathPrefix << std::endl;
        }
    }
}

void TaskOptimization::insertObstacle()
{
    Eigen::Vector3d insertPosition;
    if (runObstacleTest_1D) {
        insertPosition << -0.25, 0, 0;
    }else if (runObstacleTest_3D) {
        insertPosition = obstacle3DGazeboPosition;
    }

    yarp::os::Bottle obstacleBottle;
    bottleEigenVector(obstacleBottle, insertPosition);
    obstacle_port.write(obstacleBottle);
}
void TaskOptimization::removeObstacle()
{
    Eigen::Vector3d removedPosition(0.4, 0, 0);

    yarp::os::Bottle obstacleBottle;
    bottleEigenVector(obstacleBottle, removedPosition);
    obstacle_port.write(obstacleBottle);
}

void TaskOptimization::executeTrajectory(double relativeTime,  ocra::Model& model)
{
    if (relativeTime>=obstacleTime) {
        insertObstacle();
    }
    else{
        removeObstacle();
    }

    // Claculate cost at timestep and write to file if logging.
    calculateInstantaneousCost(relativeTime, model, rHandIndex);
    if(logTrajectoryData)
    {
        realTrajectoryFile << relativeTime << " "
                            << rightHandTask->getTaskFramePosition().transpose() << " "
                            << rightHandTask->getTaskFrameLinearVelocity().transpose() << " "
                            << rightHandTask->getTaskFrameLinearAcceleration().transpose() << " "
                            << rightHandTask->getWeights().transpose()
                            << std::endl;
    }


    rightHandTrajectory->getDesiredValues(relativeTime, desiredPosVelAcc_rightHand, desiredVariance_rightHand);

    rightHandTask->setState(desiredPosVelAcc_rightHand.col(0));

    if (useVarianceModulation)
    {
        Eigen::VectorXd desiredWeights_rightHand_tmp = mapVarianceToWeights(desiredVariance_rightHand);
        if (runObstacleTest_1D)
        {
            desiredWeights_rightHand = Eigen::VectorXd::Ones(3);
            desiredWeights_rightHand(dofIndex) = desiredWeights_rightHand_tmp(dofIndex);
        }
        else if(runObstacleTest_3D)
        {
            desiredWeights_rightHand = desiredWeights_rightHand_tmp;
        }

        rightHandTask->setWeights(desiredWeights_rightHand);

    }



}

bool TaskOptimization::sendTestDataToSolver()
{
    double totalCost = postProcessInstantaneousCosts();

    yarp::os::Bottle& optVarsBottle = optVarsPortOut.prepare();
    bottleEigenVector(optVarsBottle, optVariables);
    optVarsPortOut.write();

    yarp::os::Bottle& costBottle = costPortOut.prepare();
    costBottle.clear();
    costBottle.addDouble(totalCost);
    costPortOut.write();

    return true;
}

double TaskOptimization::postProcessInstantaneousCosts()
{
    Eigen::MatrixXd totalCostMat = Eigen::MatrixXd::Zero(costIterCounter,2);

    if (useGoalCost){
        goalCostMat = goalCostMat.topRows(costIterCounter).eval();
        totalCostMat.col(1) += goalCostMat.col(1);
        totalCostMat.col(0) = goalCostMat.col(0);
    }
    if (useTrackingCost){
        trackingCostMat = trackingCostMat.topRows(costIterCounter).eval();
        totalCostMat.col(1) += trackingCostMat.col(1);
        totalCostMat.col(0) = trackingCostMat.col(0);
    }
    if (useEnergyCost){
        energyCostMat = energyCostMat.topRows(costIterCounter).eval();
        if (testNumber==1) {
            energyCostScalingFactor = energyCostMat.col(1).maxCoeff();
        }
        energyCostMat.col(1) /= energyCostScalingFactor;

        totalCostMat.col(1) += energyCostMat.col(1);
        totalCostMat.col(0) = energyCostMat.col(0);
    }


    if (logTrajectoryData) {

        if (useGoalCost){
            goalInstantaneousCostFile << goalCostMat;
        }
        if (useTrackingCost){
            trackingInstantaneousCostFile << trackingCostMat;
        }
        if (useEnergyCost){
            energyInstantaneousCostFile << energyCostMat;
        }
        totalInstantaneousCostFile << totalCostMat;
    }

    double timeExecution = totalCostMat.maxCoeff();
    double returnCost = totalCostMat.col(1).sum() / timeExecution;

    return returnCost;
}


bool TaskOptimization::parseNewOptVarsBottle()
{
    yarp::os::Bottle *newOptVars = optVarsPortIn.read(false);
    if (newOptVars!=NULL)
    {
        optimumFound = bool(newOptVars->get(0).asInt());
        if (optimumFound) {
            std::cout << "\n---------------\nFound optimum!\n---------------\n" << std::endl;
            std::cout <<"[OPTIMAL VARIABLES]\n"<< newOptVars->toString() <<"\n"<< std::endl;
        }else{
            std::cout <<"[NEW TEST VARIABLES]\n"<< newOptVars->toString() <<"\n"<< std::endl;
        }

        for(int i=0; i<optVariables.size(); i++)
        {
          optVariables(i) = newOptVars->get(i+1).asDouble();
        }
        rightHandTrajectory->setBoptVariables(optVariables);
        if (runObstacleTest_1D) {
            currentOptWaypoint(dofIndex) = optVariables(1);
        }else if (runObstacleTest_3D) {
            currentOptWaypoint = optVariables.tail(3);
        }
        return true;
    }
    else{return false;}
}

void TaskOptimization::doUpdate(double time, ocra::Model& model, void** args)
{

    sendFramePositionsToGazebo();

    if (runObstacleTest_1D || runObstacleTest_3D) {
        obstacleTest_UpdateThread(time, model);
    }
    else if (runArmCrossingTest) {
        ArmCrossingTest_UpdateThread(time, model);
    }







}

void TaskOptimization::sendFramePositionsToGazebo()
{
    Eigen::Vector3d currentRightHandPos = (rightHandTask->getTaskFramePosition() + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1, -1, 1);

    yarp::os::Bottle r_hand_output;
    bottleEigenVector(r_hand_output, currentRightHandPos);
    r_hand_port.write(r_hand_output);

    Eigen::Vector3d currentOptWaypoint_transformed = (currentOptWaypoint + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);

    yarp::os::Bottle r_hand_waypoint_output;
    bottleEigenVector(r_hand_waypoint_output, currentOptWaypoint_transformed);
    r_hand_waypoint_port.write(r_hand_waypoint_output);

    yarp::os::Bottle r_hand_start_output;
    bottleEigenVector(r_hand_start_output, rightHandStartPosition_transformed);
    r_hand_start_port.write(r_hand_start_output);

    yarp::os::Bottle r_hand_target_output;
    bottleEigenVector(r_hand_target_output, rightHandGoalPosition_transformed);
    r_hand_target_port.write(r_hand_target_output);
}




void TaskOptimization::bottleEigenVector(yarp::os::Bottle& bottle, const Eigen::VectorXd& vecToBottle, const bool encapsulate)
{
    bottle.clear();
    for(int i =0; i<vecToBottle.size(); i++){
        bottle.addDouble(vecToBottle(i));
    }
}

// void encapsulateBottleData()

bool TaskOptimization::isBackInHomePosition(ocra::Model& model, int segmentIndex)
{
    double error;
    Eigen::Vector3d currentDesiredPosition, taskFrame;
    if (segmentIndex==rHandIndex) {
        taskFrame = rightHandTask->getTaskFramePosition();
    }
    else{
        std::cout << "[ERROR] TaskOptimization::attainedGoal - segment name doesn't match either l_hand or r_hand" << std::endl;
        return false;
    }

    error = (rHandPosStart - taskFrame ).norm();
    bool result = error <= ERROR_THRESH;
    return result;
}

bool TaskOptimization::attainedGoal(ocra::Model& model, int segmentIndex)
{
    double error;
    Eigen::Vector3d currentDesiredPosition, taskFrame;
    if (segmentIndex==rHandIndex) {
        currentDesiredPosition = rightHandGoalPosition;
        taskFrame = rightHandTask->getTaskFramePosition();
    }
    else{
        std::cout << "[ERROR] TaskOptimization::attainedGoal - segment name doesn't match either l_hand or r_hand" << std::endl;
        return false;
    }

    error = (currentDesiredPosition - taskFrame ).norm();
    bool result = error <= ERROR_THRESH;
    return result;
}


Eigen::VectorXd TaskOptimization::mapVarianceToWeights(Eigen::VectorXd& variance)
{
    double beta = 1.0;
    variance /= maxVariance;
    variance = variance.array().min(varianceThresh); //limit variance to 0.99 maximum
    Eigen::VectorXd weights = (Eigen::VectorXd::Ones(variance.rows()) - variance) / beta;
    return weights;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskOptimization::calculateInstantaneousCost(const double time, const ocra::Model& model, int segmentIndex)
{

    if (useGoalCost){
        goalCostMat.row(costIterCounter) << time, calculateGoalCost(time, model, segmentIndex);
    }
    if (useTrackingCost){
        trackingCostMat.row(costIterCounter) << time, calculateTrackingCost(time, model, segmentIndex);
    }
    if (useEnergyCost){
        energyCostMat.row(costIterCounter) << time, calculateEnergyCost(time, model, segmentIndex);
    }

    costIterCounter++;

}

double TaskOptimization::calculateGoalCost(const double time, const ocra::Model& model, int segmentIndex)
{
    double cost = ( rightHandGoalPosition - rightHandTask->getTaskFramePosition() ).squaredNorm();
    double timeFactor = pow((time / rightHandTrajectory->getDuration()), 10);

    cost *= timeFactor;
    return cost;
}


double TaskOptimization::calculateTrackingCost(const double time, const ocra::Model& model, int segmentIndex)
{
    double cost = ( desiredPosVelAcc_rightHand.col(0) - rightHandTask->getTaskFramePosition() ).squaredNorm();
    return cost;
}


double TaskOptimization::calculateEnergyCost(const double time, const ocra::Model& model, int segmentIndex)
{
    Eigen::VectorXd torques = model.getJointTorques();
    return torques.squaredNorm();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
*   1D Obstacle Test
*/
void TaskOptimization::obstacleTest_UpdateThread(double time, ocra::Model& model)
{
    if(!sequenceFinished)
    {

        if (initTrigger) {
            std::cout << "initTraj for test number = " << testNumber << std::endl;
            initializeTrajectory(time);
            std::cout << "Done..." << std::endl;
        }


        if(!optimumFound || (newOptVarsReceived && waitForHomePosition ) )
        {
            if(!waitForSolver)
            {
                double relativeTime = time - resetTimeRight;

                if ( (std::abs(relativeTime) <= TIME_LIMIT) && !attainedGoal(model, rHandIndex))
                {
                    executeTrajectory(relativeTime, model);
                }
                else
                {
                    removeObstacle();
                    if((std::abs(relativeTime) > TIME_LIMIT)){
                        std::cout << "Time limit exceeded!" << std::endl;
                    }
                    if (attainedGoal(model, rHandIndex)) {
                        std::cout << "Goal attained!" << std::endl;
                    }

                    std::cout << "Going to starting position..." << std::endl;
                    rightHandTask->setState(rHandPosStart);
                    rightHandTask->setWeights(Eigen::Vector3d::Ones(3));

                    waitForSolver = true;
                }
            }
            else
            {
                removeObstacle();

                if (!dataSent_AwaitReply)
                {
                    dataSent_AwaitReply = sendTestDataToSolver();
                    std::cout << "Data sent to solver, awaiting new optimal waypoint variables..." << std::endl;
                }
                else
                {
                    if(!newOptVarsReceived)
                    {
                            newOptVarsReceived = parseNewOptVarsBottle();
                            waitCount = 0;
                            waitForHomePosition = true;
                    }
                    else
                    {
                        if(isBackInHomePosition(model, rHandIndex))
                        {
                            dataSent_AwaitReply = false;
                            waitForSolver = false;
                            newOptVarsReceived = false;
                            initTrigger = true;
                            waitForHomePosition = false;
                            if(logTrajectoryData)
                            {
                                if (!closeLogFiles()) {
                                std::cout << "[ERROR](line: "<< __LINE__ << ") -> Could not close log files for test number: " << testNumber << std::endl;
                                }
                            }
                            testNumber++;
                        }
                        else
                        {
                            if (waitCount>=100)
                            {
                                rightHandTask->setState(rHandPosStart);
                                rightHandTask->setWeights(Eigen::Vector3d::Ones(3));

                                std::cout << "New test variables received, waiting for robot to return to home position." << std::endl;
                                waitCount = 0;
                            }
                            waitCount++;
                        }
                    }

                }
            }
        }
        else
        {
            double relativeTime = time - resetTimeRight;

            if ( (std::abs(relativeTime) <= TIME_LIMIT) && !attainedGoal(model, rHandIndex) && !waitForHomePosition)
            {
                executeTrajectory(relativeTime, model);
            }
            else
            {
                postProcessInstantaneousCosts();
                if(logTrajectoryData)
                {
                    if (!closeLogFiles()) {
                    std::cout << "[ERROR](line: "<< __LINE__ << ") -> Could not close log files for test number: " << testNumber << std::endl;
                    }
                }

                removeObstacle();


                if (replayOptimalTrajectory)
                {
                    rightHandTask->setState(rHandPosStart);
                    rightHandTask->setWeights(Eigen::Vector3d::Ones(3));
                    waitForHomePosition = true;

                    if(isBackInHomePosition(model, rHandIndex))
                    {
                        initTrigger = true;
                        waitForHomePosition = false;
                    }
                    logTrajectoryData = false;
                }
                else
                {
                    sequenceFinished = true;
                    std::cout   << "\n==========================================================\n"
                                << "\tTaskOptimization sequence finished!"
                                << "\n==========================================================\n"
                                << std::endl;
                }
            }



        }
    }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
*   3D Obstacle Test
*/

void TaskOptimization::ArmCrossingTest_UpdateThread(double time, ocra::Model& model)
{
    /* code */
}
