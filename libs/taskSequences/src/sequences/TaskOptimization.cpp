#include <taskSequences/sequences/TaskOptimization.h>
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

TaskOptimization::TaskOptimization()
{
    connectYarpPorts();


    useGoalCost = false;

    useTrackingCost = true;

    useEnergyCost = false;

    logTrajectoryData = true;

    rootLogFilePathPrefix = "/home/ryan/Desktop/tmp-test/A";

}

TaskOptimization::~TaskOptimization()
{
    optVarsPortOut.close();
    costPortOut.close();
    optVarsPortIn.close();
    optParamsPortOut.close();

    r_hand_port.close();
    r_hand_target_port.close();
    r_hand_waypoint_port.close();



}

bool TaskOptimization::openLogFiles(const std::string testLogFilePathPrefix)
{
    bool retVal = true;
    if (logTrajectoryData)
    {
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

    std::string r_hand_target_port_name = "/rHandTarget:o";
    r_hand_target_port.open(r_hand_target_port_name.c_str());
    yarp.connect(r_hand_target_port_name.c_str(), "/rightHandTargetSphere:i");

    std::string r_hand_waypoint_port_name = "/rHandWaypoint:o";
    r_hand_waypoint_port.open(r_hand_waypoint_port_name.c_str());
    yarp.connect(r_hand_waypoint_port_name.c_str(), "/rightHandWaypointSphere:i");
}

void TaskOptimization::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
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


    //  rightHand
    double Kp_rightHand = 20.0;
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


    //  rightHand
    Eigen::Vector3d r_handDisp(0.05, 0.0, 0.0); // Moves the task frame to the center of the hand.
    taskManagers["rightHand"] = new wocra::wOcraVariableWeightsTaskManager(ctrl, model, "rightHand", "r_hand", r_handDisp, Kp_rightHand, Kd_rightHand, weights_rightHand, usesYARP);


    /*
    *   Trajectory constructors
    */

    rightHandTrajectory = new wocra::wOcraGaussianProcessTrajectory();


    /*
    *   Cast tasks to derived classes to access their virtual functions
    */
    rightHandTask = dynamic_cast<wocra::wOcraVariableWeightsTaskManager*>(taskManagers["rightHand"]);


    /*
    *   Variables used in the doUpdate control logic
    */
    rHandIndex = model.getSegmentIndex("r_hand");

    initTrigger = true;



    //Figure out waypoints
    rHandPosStart = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    rHandPosStart(0) = -0.35;
    rHandPosStart(1) = 0.2;
    rHandPosStart(2) = 0.0;


    dofIndex = 2;
    Eigen::Vector3d rHandDisplacement = Eigen::Vector3d::Zero();
    rHandDisplacement(dofIndex) = 0.25; // meters
    rHandPosEnd = rHandPosStart + rHandDisplacement;

    std::cout << "\n\n\n rHandPosStart = "<<rHandPosStart.transpose() << "\n rHandPosEnd = " << rHandPosEnd.transpose() << "  \n\n\n" << std::endl;

    rightHandTrajectory->setWaypoints(rHandPosStart, rHandPosEnd);

    desiredPosVelAcc_rightHand = Eigen::MatrixXd::Zero(3,3);
    desiredPosVelAcc_rightHand.col(0) << rHandPosStart;

    rightHandGoalPosition = rHandPosEnd;
    rightHandGoalPosition_transformed = (rightHandGoalPosition + Eigen::Vector3d(0,0,1)).array() * Eigen::Array3d(-1,-1,1);


    std::vector<Eigen::VectorXi> dofToOptimize(1);
    dofToOptimize[0].resize(2);
    dofToOptimize[0] << 0, dofIndex+1;
    optVariables = rightHandTrajectory->getBoptVariables(1, dofToOptimize);

    std::cout << "optVariables\n" << optVariables << std::endl;

    currentOptWaypoint = rHandPosEnd;
    currentOptWaypoint(dofIndex) = optVariables(1);





    maxVariance = rightHandTrajectory->getMaxVariance();

    waitForSolver = false;
    waitForHomePosition = false;
    optimumFound = false;



    // Send opt params
    sendOptimizationParameters();

    newOptVarsReceived = false;
    dataSent_AwaitReply = false;
    testNumber = 0;


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

    optParamsPortOut.write();
}


void TaskOptimization::doUpdate(double time, wocra::wOcraModel& state, void** args)
{

    sendFramePositionsToGazebo();

    if (initTrigger) {
        resetTimeRight = time;
        initTrigger = false;
        totalCost = 0.0;
        if (logTrajectoryData) {
            // Convert an int to a string pre C++11 safe
            std::ostringstream intAsStream; intAsStream << testNumber;
            std::string testLogFilePathPrefix = intAsStream.str();
            // Append log file path with test number
            testLogFilePathPrefix = rootLogFilePathPrefix +"/"+ testLogFilePathPrefix;
            checkAndCreateDirectory(testLogFilePathPrefix);
            if(!openLogFiles(testLogFilePathPrefix))
            {
                std::cout << "[ERROR](line: "<< __LINE__ <<") -> Could not open data log files for the trajectory! Tried: "<< testLogFilePathPrefix << std::endl;
            }
        }
    }


    if(!optimumFound)
    {
        if(!waitForSolver)
        {
            double relativeTime = time - resetTimeRight;

            if ( (abs(relativeTime) <= TIME_LIMIT) && !attainedGoal(state, rHandIndex))
            {

                // Claculate cost at timestep and write to file if logging.
                double instantaneousCost = calculateInstantaneousCost(time, state, rHandIndex);
                totalCost += instantaneousCost;
                if(logTrajectoryData)
                {totalInstantaneousCostFile << relativeTime << " " << instantaneousCost << std::endl;}


                rightHandTrajectory->getDesiredValues(relativeTime, desiredPosVelAcc_rightHand, desiredVariance_rightHand);
                Eigen::VectorXd desiredWeights_rightHand_tmp = mapVarianceToWeights(desiredVariance_rightHand);
                desiredWeights_rightHand = Eigen::VectorXd::Ones(3);
                desiredWeights_rightHand(dofIndex) = desiredWeights_rightHand_tmp(dofIndex);

                rightHandTask->setState(desiredPosVelAcc_rightHand.col(0));
                rightHandTask->setWeights(desiredWeights_rightHand);


            }
            else
            {
                if((abs(relativeTime) > TIME_LIMIT)){
                    std::cout << "Time limit exceeded!" << std::endl;
                }
                if (attainedGoal(state, rHandIndex)) {
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
            if (!dataSent_AwaitReply)
            {
                yarp::os::Bottle& optVarsBottle = optVarsPortOut.prepare();
                bottleEigenVector(optVarsBottle, optVariables);
                optVarsPortOut.write();

                yarp::os::Bottle& costBottle = costPortOut.prepare();
                costBottle.clear();
                costBottle.addDouble(totalCost);
                costPortOut.write();

                dataSent_AwaitReply = true;
                std::cout << "Data sent to solver, awaiting new optimal waypoint variables..." << std::endl;
            }
            else
            {
                if(!newOptVarsReceived)
                {
                    yarp::os::Bottle *newOptVars = optVarsPortIn.read(false);
                    if (newOptVars!=NULL)
                    {
                        std::cout <<"[NEW TEST VARIABLES]\n"<< newOptVars->toString() <<"\n"<< std::endl;
                        optimumFound = bool(newOptVars->get(0).asInt());
                        if (optimumFound) {
                            std::cout << "--> Found optimal waypoint variables! Replaying till thread is killed." << std::endl;
                        }

                        for(int i=0; i<optVariables.size(); i++)
                        {
                          optVariables(i) = newOptVars->get(i+1).asDouble();
                        }
                        rightHandTrajectory->setBoptVariables(optVariables);
                        currentOptWaypoint(dofIndex) = optVariables(1);
                        newOptVarsReceived = true;
                        waitCount = 0;
                    }
                }
                else
                {
                    if(isBackInHomePosition(state, rHandIndex))
                    {
                        dataSent_AwaitReply = false;
                        waitForSolver = false;
                        newOptVarsReceived = false;
                        initTrigger = true;
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

        if ( (abs(relativeTime) <= TIME_LIMIT) && !attainedGoal(state, rHandIndex) && !waitForHomePosition)
        {
            rightHandTrajectory->getDesiredValues(relativeTime, desiredPosVelAcc_rightHand, desiredVariance_rightHand);
            Eigen::VectorXd desiredWeights_rightHand_tmp = mapVarianceToWeights(desiredVariance_rightHand);
            desiredWeights_rightHand = Eigen::VectorXd::Ones(3);
            desiredWeights_rightHand(dofIndex) = desiredWeights_rightHand_tmp(dofIndex);

            rightHandTask->setState(desiredPosVelAcc_rightHand.col(0));
            rightHandTask->setWeights(desiredWeights_rightHand);


        }
        else
        {
            if((abs(relativeTime) > TIME_LIMIT)){
                std::cout << "Time limit exceeded!" << std::endl;
            }
            if (attainedGoal(state, rHandIndex)) {
                std::cout << "Goal attained!" << std::endl;
            }

            std::cout << "Going to starting position..." << std::endl;
            rightHandTask->setState(rHandPosStart);
            rightHandTask->setWeights(Eigen::Vector3d::Ones(3));
            waitForHomePosition = true;

            if(isBackInHomePosition(state, rHandIndex))
            {
                initTrigger = true;
                waitForHomePosition = false;

            }
        }
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

bool TaskOptimization::isBackInHomePosition(wocra::wOcraModel& state, int segmentIndex)
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

bool TaskOptimization::attainedGoal(wocra::wOcraModel& state, int segmentIndex)
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
double TaskOptimization::calculateInstantaneousCost(const double time, const wocra::wOcraModel& state, int segmentIndex)
{
    double returnCost = 0.0;

    if (useGoalCost){
        double goalCost = calculateGoalCost(time, state, segmentIndex);
        if (logTrajectoryData) {
            goalInstantaneousCostFile << time << goalCost << std::endl;
        }
        returnCost += goalCost;
    }
    if (useTrackingCost){
        double trackingCost = calculateTrackingCost(time, state, segmentIndex);
        if (logTrajectoryData) {
            trackingInstantaneousCostFile << time << trackingCost << std::endl;
        }
        returnCost += trackingCost;
    }
    if (useEnergyCost){
        double energyCost = calculateEnergyCost(time, state, segmentIndex);
        if (logTrajectoryData) {
            energyInstantaneousCostFile << time << energyCost << std::endl;
        }
        returnCost += energyCost;
    }

    return returnCost;
}

double TaskOptimization::calculateGoalCost(const double time, const wocra::wOcraModel& state, int segmentIndex)
{
    double cost = ( rightHandGoalPosition - rightHandTask->getTaskFramePosition() ).squaredNorm();
    return cost;
}


double TaskOptimization::calculateTrackingCost(const double time, const wocra::wOcraModel& state, int segmentIndex)
{
    double cost = ( desiredPosVelAcc_rightHand.col(0) - rightHandTask->getTaskFramePosition() ).squaredNorm();
    return cost;
}


double TaskOptimization::calculateEnergyCost(const double time, const wocra::wOcraModel& state, int segmentIndex)
{
    double cost = 0.0;

    return cost;
}
