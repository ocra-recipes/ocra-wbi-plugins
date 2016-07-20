
#include <taskSequences/sequences/MoveWeight.h>

#ifndef ERROR_THRESH
#define ERROR_THRESH 0.03 // Goal error threshold for hand tasks
#endif

#ifndef VAR_THRESH
#define VAR_THRESH 0.5
#endif

#ifndef VAR_BETA
#define VAR_BETA 20.0 //50
#endif

#ifndef TIME_LIMIT
#define TIME_LIMIT 15.0 // Maximum time to be spent on any trajectory.
#endif

#ifndef DEACTIVATION_TIME_LIMIT
#define DEACTIVATION_TIME_LIMIT 4.0 // Maximum time to be spent on any trajectory.
#endif


#ifndef ACTIVATION_TIME_LIMIT
#define ACTIVATION_TIME_LIMIT 8.0 // Maximum time to be spent on any trajectory.
#endif

#ifndef STABILIZATION_TIME
#define STABILIZATION_TIME 3.0 // Maximum time to be spent on any trajectory.
#endif


MoveWeight::MoveWeight()
{

    useVarianceModulation = false;

    connectYarpPorts();

    bOptCovarianceScalingFactor = 40.0;

    replayOptimalTrajectory = true;

    useGoalCost = true;
    useTrackingCost = true;
    useEnergyCost = true;


    logTrajectoryData = true;

    rootLogFilePathPrefix = "./MoveWeight/raw_data/";

    rootLogFilePathPrefix += "MoveWeight";

    if (useVarianceModulation)
    {
        rootLogFilePathPrefix += "_VarianceModulated";
    }

    smlt::checkAndCreateDirectory(rootLogFilePathPrefix);


}

MoveWeight::~MoveWeight()
{
    optVarsPortOut.close();
    costPortOut.close();
    optVarsPortIn.close();
    optParamsPortOut.close();
    rpcClientPort.close();

    r_hand_port.close();
    r_hand_target_port.close();
    r_hand_start_port.close();
    r_hand_waypoint_port.close();

}


void MoveWeight::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    ocraWbiModel& wbiModelRef = dynamic_cast<ocraWbiModel&>(model);
    wbiModel = &wbiModelRef;

    varianceThresh = Eigen::Array3d::Constant(VAR_THRESH);

    bool usesYARP = true;

    double Kp_fullPosture       = 5.0;
    double Kd_fullPosture       = 2.0 * sqrt(Kp_fullPosture);
    double weight_fullPosture   = 0.0001;

    //  CoM
    double Kp_CoM      = 50.0;
    double Kd_CoM      = 1.0 * sqrt(Kp_CoM);
    // double weight_CoM  = 0.001;
    double weight_CoM  = 1.0;

    //  torsoCartesian
    double Kp_torso      = 1.0;
    double Kd_torso      = 2.0 * sqrt(Kp_torso);
    // double weight_CoM  = 0.001;
    double weight_torso  = 0.001;

    //  rootCartesian
    double Kp_root      = 1.0;
    double Kd_root      = 2.0 * sqrt(Kp_root);
    // double weight_CoM  = 0.001;
    double weight_root  = 0.001;


    //  rightHand
    double Kp_rightHand = 70.0;
    double Kd_rightHand = 2.0 *sqrt(Kp_rightHand);


    // rightHandStaticWeight = Eigen::VectorXd::Ones(6) / VAR_BETA;
    rightHandStaticWeight = Eigen::VectorXd::Ones(3) / VAR_BETA;
    Eigen::VectorXd weights_rightHand = rightHandStaticWeight;
    desiredWeights_rightHand.resize(weights_rightHand.size());
    // rightHandStaticWeight(3) = 0.0;
    // rightHandStaticWeight(4) = 0.0;
    // rightHandStaticWeight(5) = 0.0;

    // Initialise full posture task
    Eigen::VectorXd q_full = wbiModel->getJointPositions();
    // q_full[wbiModel->getDofIndex("l_knee")] = -PI / 3;
    // q_full[wbiModel->getDofIndex("r_knee")] = -PI / 3;

    // q_full[wbiModel->getDofIndex("l_shoulder_roll")] = PI / 6;


    Eigen::VectorXd w_full = Eigen::VectorXd::Constant(wbiModel->nbInternalDofs(), weight_fullPosture);

    w_full[wbiModel->getDofIndex("l_hip_pitch")] = 0.0004; //0.0004
    w_full[wbiModel->getDofIndex("l_hip_roll")] = 0.1;
    w_full[wbiModel->getDofIndex("l_hip_yaw")] = 0.1;
    // w_full[wbiModel->getDofIndex("l_knee")] = ;
    // w_full[wbiModel->getDofIndex("l_ankle_roll")] = ;
    w_full[wbiModel->getDofIndex("r_hip_pitch")] = 0.0004; //0.0004
    w_full[wbiModel->getDofIndex("r_hip_roll")] = 0.1;
    w_full[wbiModel->getDofIndex("r_hip_yaw")] = 0.1;
    // w_full[wbiModel->getDofIndex("r_knee")] = ;
    // w_full[wbiModel->getDofIndex("r_ankle_roll")] = ;

    w_full[wbiModel->getDofIndex("l_ankle_pitch")] = 1.0;
    w_full[wbiModel->getDofIndex("r_ankle_pitch")] = 1.0;



    w_full[wbiModel->getDofIndex("l_knee")] = 0.0004; //0.0004
    w_full[wbiModel->getDofIndex("r_knee")] = 0.0004; //0.0004


    w_full[wbiModel->getDofIndex("torso_pitch")] = 0.1;
    w_full[wbiModel->getDofIndex("torso_roll")] = 0.01;
    w_full[wbiModel->getDofIndex("torso_yaw")] = 0.001;

    // w_full[wbiModel->getDofIndex("r_shoulder_pitch")] = 0.00001;
    // w_full[wbiModel->getDofIndex("r_shoulder_roll")] = 0.00001;
    w_full[wbiModel->getDofIndex("r_shoulder_yaw")] = 0.01;
    w_full[wbiModel->getDofIndex("r_elbow")] = 0.0001;
    w_full[wbiModel->getDofIndex("l_elbow")] = 0.00001;

    // w_full[wbiModel->getDofIndex("l_shoulder_roll")] = 1.0;


    taskManagers["fullPostureTask"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp_fullPosture, Kd_fullPosture, w_full, q_full, usesYARP);



    // Initialise com task
    initialCoMPosition = wbiModel->getCoMPosition();
    taskManagers["CoMTask"] = new wocra::wOcraCoMTaskManager(ctrl, model, "CoMTask", ocra::XY, Kp_CoM, Kd_CoM, weight_CoM, initialCoMPosition, usesYARP);
    comTask = dynamic_cast<wocra::wOcraCoMTaskManager*>(taskManagers["CoMTask"]);


    // Initialise torso pose
    Eigen::Vector3d desiredTorsoPosition, XYZdisp;
    desiredTorsoPosition = wbiModel->getSegmentPosition(wbiModel->getSegmentIndex("torso")).getTranslation();
    XYZdisp << 0.0, 0.0, 0.0;
    desiredTorsoPosition = desiredTorsoPosition + XYZdisp;
    taskManagers["torsoCartesianTask"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "torsoCartesian", "torso", ocra::XY, Kp_torso, Kd_torso, weight_torso, desiredTorsoPosition, usesYARP);

    // Initialise root pose
    // Eigen::Vector3d desiredRootPosition;
    // desiredRootPosition = wbiModel->getSegmentPosition(wbiModel->getSegmentIndex("root_link")).getTranslation();
    // // XYZdisp << 0.0, 0.0, 0.0;
    // desiredRootPosition = desiredRootPosition + XYZdisp;
    // taskManagers["rootCartesianTask"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rootCartesian", "root_link", ocra::XY, Kp_root, Kd_root, weight_root, desiredRootPosition, usesYARP);



    // Initialise foot contacts
    double mu_sys = 1.0;
    double margin = 0.05;
    double sqrt2on2 = sqrt(2.0)/2.0;

    Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(sqrt2on2, 0.0, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
    Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, 0.0, -sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);


    std::vector<Eigen::Displacementd> LFContacts;
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotLZdown));
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotLZdown));
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotLZdown));
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotLZdown));
    taskManagers["leftFootContactTask"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_sole", LFContacts, mu_sys, margin);

    std::vector<Eigen::Displacementd> RFContacts;
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotRZdown));
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotRZdown));
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotRZdown));
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotRZdown));
    taskManagers["rightFootContactTask"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "rightFootContactTask", "r_sole", RFContacts, mu_sys, margin);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    //  rightHand
    double Kp_rightHandOrientation = 80.0;
    double Kd_rightHandOrientation = 2.0 *sqrt(Kp_rightHandOrientation);
    Eigen::Vector3d weight_rightHandOrientation(0.0, 1.0, 0.0);

    Eigen::Rotation3d startingRotd = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getRotation();
    // std::cout << model.getSegmentPosition(model.getSegmentIndex("l_sole")) << std::endl;
    // Eigen::Rotation3d yToNegZ = Eigen::Rotation3d(sqrt2on2, -sqrt2on2, 0.0, 0.0);
    Eigen::Rotation3d desiredRightHandOrientation = startingRotd;//.inverse();// * yToNegZ;

    taskManagers["rightHandOrientationTask"]    = new wocra::wOcraSegOrientationTaskManager(ctrl, model, "rightHandOrientationTask", "r_hand", Kp_rightHandOrientation, Kd_rightHandOrientation, weight_rightHandOrientation, desiredRightHandOrientation, usesYARP);

    Eigen::Vector3d r_handDisp(0.085, 0.04, 0.045); // Moves the task frame to the center of the hand.
    taskManagers["rightHand"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHand", "r_hand", r_handDisp, ocra::XYZ, Kp_rightHand, Kd_rightHand, weights_rightHand, usesYARP);
    rightHandTask = dynamic_cast<wocra::wOcraSegCartesianTaskManager*>(taskManagers["rightHand"]);




    // //  rightHand
    // Eigen::Displacementd r_handDisp(0.085, 0.04, 0.045, 1.0, 0.0, 0.0, 0.0); // Moves the task frame to the center of the hand.
    // taskManagers["rightHand"] = new wocra::wOcraSegPoseTaskManager(ctrl, model, "rightHand", "r_hand", r_handDisp, ocra::XYZ, Kp_rightHand, Kd_rightHand, weights_rightHand, usesYARP);
    // Cast tasks to derived classes to access their virtual functions
    // rightHandTask = dynamic_cast<wocra::wOcraSegPoseTaskManager*>(taskManagers["rightHand"]);


    // Trajectory constructor
    rightHandTrajectory = new wocra::wOcraGaussianProcessTrajectory();



    rHandPosStart = rightHandTask->getTaskFramePosition();
    Eigen::Vector3d rHandDisplacement;

    rHandDisplacement << 0.05, 0.05, 0.0;
    rHandPosEnd = rHandPosStart + rHandDisplacement;
    rHandPosEnd(2) = 0.41;


    // Set waypoints and traj
    rightHandTrajectory->setWaypoints(rHandPosStart, rHandPosEnd);



    /*
    *   Variables used in the doUpdate control logic
    */
    waitForSolver = false;
    waitForHomePosition = false;
    optimumFound = false;
    sequenceFinished = false;
    initTrigger = true;
    newOptVarsReceived = false;
    dataSent_AwaitReply = false;
    testNumber = 1;
    initializeStabilization=true;
    deactivatingHandTask=true;
    movingToWeight = true;
    optimizationInProgress = false;
    printedOnce = false;


    /*
    *   For gazebo
    */
    gazeboTranslation << 0.0, 0.0681, 0.0;


}


void MoveWeight::setInitialWaypoints()
{
    // rHandPosStart = rightHandTask->getTaskFramePosition();
    rHandPosStart = rHandPosEnd;

    Eigen::Vector3d rHandDisplacement, rHandPosMiddle;
    rHandDisplacement << 0.1, 0.1, 0.05;
    rHandPosMiddle = rHandPosStart + rHandDisplacement;
    rHandDisplacement << 0.15, 0.25, 0.0;
    rHandPosEnd = rHandPosStart + rHandDisplacement;

    Eigen::MatrixXd trajWaypoints(3,3);
    trajWaypoints.col(0) << rHandPosStart;
    trajWaypoints.col(1) << rHandPosMiddle;
    trajWaypoints.col(2) << rHandPosEnd;


    // Set waypoints and traj
    rightHandTrajectory->setMaxVelocity(0.05); // default is 0.1 m/s
    rightHandTrajectory->setWaypoints(trajWaypoints);


    /*
    *   Set opt variables
    */
    // std::vector<bool> isMeanWaypoint(5);
    // isMeanWaypoint[0] =     true;
    // isMeanWaypoint[1] =     true;
    // isMeanWaypoint[2] =     true;
    // testTrajectory->setMeanWaypoints(isMeanWaypoint);
    //
    // std::vector<bool> isVarWaypoint(5);
    // isVarWaypoint[0] =      true;
    // isVarWaypoint[1] =      true;
    // isVarWaypoint[2] =      true;
    // testTrajectory->setVarianceWaypoints(isVarWaypoint);

    std::vector<bool> isOptWaypoint(3);
    isOptWaypoint[0] = false;
    isOptWaypoint[1] = true;
    isOptWaypoint[2] = false;

    // std::vector<Eigen::VectorXi> dofToOptimize(3);
    // dofToOptimize[0] = Eigen::VectorXi();
    // Eigen::VectorXi tmpVec(3);
    // tmpVec << 1,2,3; // X Y Z
    // dofToOptimize[1] = tmpVec;
    // dofToOptimize[2] = Eigen::VectorXi();

    std::vector<Eigen::VectorXi> dofToOptimize(3);
    dofToOptimize[0] = Eigen::VectorXi();
    Eigen::VectorXi tmpVec(4);
    tmpVec << 0,1,2,3; // t X Y Z
    dofToOptimize[1] = tmpVec;
    dofToOptimize[2] = Eigen::VectorXi();


    // rightHandTrajectory->setMeanWaypoints(isMeanWaypoint);
    // rightHandTrajectory->setVarianceWaypoints(isVarWaypoint);
    rightHandTrajectory->setOptimizationWaypoints(isOptWaypoint);
    rightHandTrajectory->setDofToOptimize(dofToOptimize);

    optVariables = rightHandTrajectory->getBoptVariables();






    /*
    *   Get max variance for weight calcs.
    */
    maxVariance = rightHandTrajectory->getMaxVariance();


    /*
    *   For the trajectory logging
    */
    desiredPosVelAcc_rightHand = Eigen::MatrixXd::Zero(3,3);
    desiredPosVelAcc_rightHand.col(0) << rHandPosStart;



    currentOptWaypoint = optVariables.tail(3);



}

void MoveWeight::initializeOptimization()
{
    // setInitialWaypoints();
    /*
    *   Send opt params
    */
    sendOptimizationParameters();
    /*
    *   Informative
    */
    rightHandTrajectory->printWaypointData();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*
*           Sequence update loop
*
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void MoveWeight::doUpdate(double time, wocra::wOcraModel& state, void** args)
{
    sendFramePositionsToGazebo();

    if(movingToWeight)
    {
        rightHandTrajectory->getDesiredValues(time, desiredPosVelAcc_rightHand, desiredVariance_rightHand);

        // rightHandTask->setState(Eigen::Displacementd(desiredPosVelAcc_rightHand.col(0), Eigen::Rotation3d::Identity() ));
        rightHandTask->setState(desiredPosVelAcc_rightHand.col(0));


        // When finished set up optimization stuff...
        if (attainedGoal(state))
        {
            checkSolverStatus();
            if (!optimizationInProgress) {
                std::cout << "I have reached the object, now I am initializing my optimization test." << std::endl;
                initializeOptimization();
                std::cout << "Optimization test initialization complete. Begining trials..." << std::endl;
            }
            else{
                std::cout << "Picking back up optimization trials on test number " << testNumber << "." << std::endl;
            }
            movingToWeight = false;
        }

    }
    else
    {

        if(!sequenceFinished)
        {

            if (initTrigger) {
                std::cout << "Initialize trajectory for test number = " << testNumber << "... ";
                initializeTrajectory(time);
                std::cout << " Done." << std::endl;
            }


            if(!optimumFound || (newOptVarsReceived && waitForHomePosition ) )
            {
                if(!waitForSolver)
                {
                    double relativeTime = time - resetTimeRight;

                    if ( (std::abs(relativeTime) <= TIME_LIMIT) && !attainedGoal(state))
                    {
                        executeTrajectory(relativeTime, state);
                    }
                    else
                    {
                        if((std::abs(relativeTime) > TIME_LIMIT)){
                            std::cout << "Time limit exceeded!" << std::endl;
                        }
                        if (attainedGoal(state)) {
                            std::cout << "Goal attained!" << std::endl;
                        }

                        waitForSolver = true;
                    }
                }
                else
                {
                    bool robotIsStable = returnToStablePosture(time, state);

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
                                waitForHomePosition = true;
                        }
                        else
                        {
                            if(robotIsStable)
                            {
                                dataSent_AwaitReply = false;
                                waitForSolver = false;
                                newOptVarsReceived = false;
                                initTrigger = true;
                                waitForHomePosition = false;
                                initializeStabilization = true;
                                deactivatingHandTask = true;

                                if(logTrajectoryData)
                                {
                                    if (!closeLogFiles()) {
                                    std::cout << "[ERROR](line: "<< __LINE__ << ") -> Could not close log files for test number: " << testNumber << std::endl;
                                    }
                                }
                                testNumber++;
                            }
                        }
                    }
                }
            }
            else
            {
                double relativeTime = time - resetTimeRight;
                if ( (std::abs(relativeTime) <= TIME_LIMIT) && !attainedGoal(state) && !waitForHomePosition)
                {
                    executeTrajectory(relativeTime, state);
                }
                else
                {
                    waitForHomePosition = true;
                    if (!printedOnce)
                    {
                        postProcessInstantaneousCosts();

                        if (attainedGoal(state)) {
                            std::cout << "Goal attained!" << std::endl;
                        }else{
                            std::cout << "Optimal values do not attain goal. Consider lowering the covariance scaling factor to increase cost function resolution." << std::endl;
                        }
                        printedOnce = true;
                    }
                    bool robotIsStable = returnToStablePosture(time, state);
                    // bool robotIsStable = false; // Use this to stay at goal location.
                    if(logTrajectoryData)
                    {

                        if (!closeLogFiles()) {
                        std::cout << "[ERROR](line: "<< __LINE__ << ") -> Could not close log files for test number: " << testNumber << std::endl;
                        }
                        logTrajectoryData = false;

                    }
                    if (replayOptimalTrajectory)
                    {
                        if(robotIsStable)
                        {
                            std::cout << "Replaying optimal trajectory." << std::endl;
                            initTrigger = true;
                            initializeStabilization = true;
                            waitForHomePosition = false;
                            printedOnce = false;
                            deactivatingHandTask = true;
                        }
                    }
                    else
                    {
                        sequenceFinished = true;
                        std::cout   << "\n==========================================================\n"
                                    << "\tObstacleAvoidance sequence finished!"
                                    << "\n==========================================================\n"
                                    << std::endl;
                    }
                }
            }
        }
    }

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*
*           Control and trajectory functions
*
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void MoveWeight::initializeTrajectory(double time)
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



void MoveWeight::executeTrajectory(double relativeTime,  wocra::wOcraModel& state)
{
    // Claculate cost at timestep and write to file if logging.
    calculateInstantaneousCost(relativeTime, state);

    if(logTrajectoryData)
    {
        realTrajectoryFile << relativeTime << " "
                            << rightHandTask->getTaskFramePosition().transpose() << " "
                            << rightHandTask->getTaskFrameLinearVelocity().transpose() << " "
                            << rightHandTask->getTaskFrameLinearAcceleration().transpose() << " "
                            << rightHandTask->getWeight().transpose()
                            << std::endl;
    }


    rightHandTrajectory->getDesiredValues(relativeTime, desiredPosVelAcc_rightHand, desiredVariance_rightHand);

    // rightHandTask->setState(Eigen::Displacementd(desiredPosVelAcc_rightHand.col(0), Eigen::Rotation3d::Identity() ));
    rightHandTask->setState(desiredPosVelAcc_rightHand.col(0));

    if (useVarianceModulation)
    {
        desiredWeights_rightHand.head(3) = mapVarianceToWeights(desiredVariance_rightHand);
        rightHandTask->setWeight(desiredWeights_rightHand);
    }



}


Eigen::VectorXd MoveWeight::mapVarianceToWeights(Eigen::VectorXd& variance)
{
    variance /= maxVariance;
    variance = variance.array().min(varianceThresh); //limit variance to 0.99 maximum
    Eigen::VectorXd weights = (Eigen::VectorXd::Ones(variance.rows()) - variance) / VAR_BETA;
    return weights;
}

bool MoveWeight::returnToStablePosture(const double time, const wocra::wOcraModel& state)
{

    bool robotIsStable = false;

    if (initializeStabilization) {
        stabilizationTime = time;
        initializeStabilization = false;
    }

    double relativeTime = time - stabilizationTime;

    if (deactivatingHandTask)
    {
        if (std::abs(relativeTime) <= DEACTIVATION_TIME_LIMIT)
        {
            double factor = (STABILIZATION_TIME - relativeTime) / STABILIZATION_TIME;
            if (factor>=0.0) {
                Eigen::VectorXd scaledWeights = rightHandStaticWeight * factor;
                rightHandTask->setWeight(scaledWeights);
            }else{
                // rightHandTask->setWeight(Eigen::VectorXd::Zero(6));
                rightHandTask->setWeight(Eigen::VectorXd::Zero(3));
                // rightHandTask->setState(Eigen::Displacementd(rHandPosStart, Eigen::Rotation3d::Identity()));
                rightHandTask->setState(rHandPosStart);

            }
        }else{
            deactivatingHandTask = false;
            initializeStabilization = true;
        }
    }
    else
    {
        if (std::abs(relativeTime) <= ACTIVATION_TIME_LIMIT)
        {
            double factor = relativeTime / STABILIZATION_TIME;
            if (factor>=0.0 && factor<=1.0) {
                Eigen::VectorXd scaledWeights = rightHandStaticWeight * factor;
                rightHandTask->setWeight(scaledWeights);
            }else{
                rightHandTask->setWeight(rightHandStaticWeight);
            }
        }else{
            robotIsStable = isBackInHomePosition(state);
        }
    }



    return robotIsStable;
}


bool MoveWeight::isBackInHomePosition(const wocra::wOcraModel& state)
{
    double error;
    error = (rHandPosStart - rightHandTask->getTaskFramePosition() ).norm();
    bool result = error <= ERROR_THRESH;
    return result;
}

bool MoveWeight::attainedGoal(const wocra::wOcraModel& state)
{
    double error;
    error = (rHandPosEnd - rightHandTask->getTaskFramePosition() ).norm();
    bool result = error <= ERROR_THRESH;
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*
*           Cost functions and post processing
*
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void MoveWeight::calculateInstantaneousCost(const double time, const wocra::wOcraModel& state)
{

    if (useGoalCost){
        goalCostMat.row(costIterCounter) << time, calculateGoalCost(time, state);
    }
    if (useTrackingCost){
        trackingCostMat.row(costIterCounter) << time, calculateTrackingCost(time, state);
    }
    if (useEnergyCost){
        energyCostMat.row(costIterCounter) << time, calculateEnergyCost(time, state);
    }

    costIterCounter++;

}

double MoveWeight::calculateGoalCost(const double time, const wocra::wOcraModel& state)
{
    double cost = ( rHandPosEnd - rightHandTask->getTaskFramePosition() ).squaredNorm();
    double timeFactor = pow((time / rightHandTrajectory->getDuration()), 10);

    cost *= timeFactor;
    return cost;
}


double MoveWeight::calculateTrackingCost(const double time, const wocra::wOcraModel& state)
{
    double cost = ( desiredPosVelAcc_rightHand.col(0) - rightHandTask->getTaskFramePosition() ).squaredNorm();
    return cost;
}


double MoveWeight::calculateEnergyCost(const double time, const wocra::wOcraModel& state)
{
    Eigen::VectorXd torques;
    wbiModel->getJointTorques(torques);
    return torques.squaredNorm();
}


double MoveWeight::postProcessInstantaneousCosts()
{
    Eigen::MatrixXd totalCostMat = Eigen::MatrixXd::Zero(costIterCounter,2);

    if (useGoalCost){
        goalCostMat = goalCostMat.topRows(costIterCounter).eval();
        totalCostMat.col(1) += goalCostMat.col(1);//*1000.0;
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
            std::ofstream energyCostScalingFactorFile;
            energyCostScalingFactorFile.open((rootLogFilePathPrefix + "/energyCostScalingFactor.txt").c_str());
            if (energyCostScalingFactorFile.is_open()) {
                energyCostScalingFactorFile << energyCostScalingFactor;
            }
            energyCostScalingFactorFile.close();

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

    // double returnCost = totalCostMat.col(1).norm();
    double timeExecution = totalCostMat.maxCoeff();
    double returnCost = totalCostMat.col(1).sum() / timeExecution;

    return returnCost;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*
*           Yarp port functions
*
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void MoveWeight::connectYarpPorts()
{
    optVarsPortOut_name = "/opt/task/vars:o";
    costPortOut_name = "/opt/task/cost:o";
    optVarsPortIn_name = "/opt/task/vars:i";
    optParamsPortOut_name = "/opt/task/params:o";

    optVarsPortOut.open(optVarsPortOut_name.c_str());
    costPortOut.open(costPortOut_name.c_str());
    optVarsPortIn.open(optVarsPortIn_name.c_str());
    optParamsPortOut.open(optParamsPortOut_name.c_str());

    std::string rpcClientPort_name = "/opt/task/rpc:c";
    rpcClientPort.open(rpcClientPort_name.c_str());

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
    connectionElapsedTime = 0.0;
    while (!yarp.connect(rpcClientPort_name.c_str(), "/opt/solver/rpc:s") && connectionElapsedTime <= connectionTimeOut ){
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


}



void MoveWeight::sendFramePositionsToGazebo()
{
    Eigen::Vector3d currentRightHandPos = rightHandTask->getTaskFramePosition() + gazeboTranslation;

    yarp::os::Bottle r_hand_output;
    bottleEigenVector(r_hand_output, currentRightHandPos);
    r_hand_port.write(r_hand_output);


    if(!movingToWeight)
    {
        yarp::os::Bottle r_hand_waypoint_output;
        bottleEigenVector(r_hand_waypoint_output, currentOptWaypoint + gazeboTranslation);
        r_hand_waypoint_port.write(r_hand_waypoint_output);
    }
    yarp::os::Bottle r_hand_start_output;
    bottleEigenVector(r_hand_start_output, rHandPosStart + gazeboTranslation);
    r_hand_start_port.write(r_hand_start_output);

    yarp::os::Bottle r_hand_target_output;
    bottleEigenVector(r_hand_target_output, rHandPosEnd + gazeboTranslation);
    r_hand_target_port.write(r_hand_target_output);
}


bool MoveWeight::sendTestDataToSolver()
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



bool MoveWeight::parseNewOptVarsBottle()
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
        currentOptWaypoint = optVariables.tail(3);
        return true;
    }
    else{return false;}
}





void MoveWeight::sendOptimizationParameters()
{
    yarp::os::Bottle& optParamsBottle = optParamsPortOut.prepare();
    optParamsBottle.clear();
    int nDims = optVariables.size();
    optParamsBottle.addInt(nDims);

    // Eigen::VectorXd searchSpaceMin = rightHandTrajectory->getBoptSearchSpaceMinBound();

    // Eigen::VectorXd searchSpaceMin(3);
    // searchSpaceMin << 0.2, -0.2, 0.26;

    // Eigen::VectorXd searchSpaceMin(4);
    // searchSpaceMin << 1.0, 0.2, -0.2, 0.26;

    // Eigen::VectorXd searchSpaceMax = rightHandTrajectory->getBoptSearchSpaceMaxBound();

    // Eigen::VectorXd searchSpaceMax(3);
    // searchSpaceMax << 0.35, -0.05, 0.5;

    // Eigen::VectorXd searchSpaceMax(4);
    // searchSpaceMax << 3.0, 0.35, -0.05, 0.5;

    Eigen::VectorXd searchSpaceMin(nDims);
    Eigen::VectorXd searchSpaceMax(nDims);

    Eigen::MatrixXd wyptData = rightHandTrajectory->getWaypointData();

    // double tMax = wyptData.col(0).maxCoeff();
    // double tMin = 0.0;
    double tMid = wyptData(1,0);
    double tMax = tMid + 1.0;
    double tMin = tMid - 1.0;



    int indexCounter = 0;

    if (nDims==4) {
        double timeBuffer = 1.5;
        searchSpaceMin(indexCounter) = tMin;
        // searchSpaceMin(indexCounter) = tMin + timeBuffer;
        searchSpaceMax(indexCounter) = tMax;
        // searchSpaceMax(indexCounter) = tMax - timeBuffer;
        indexCounter++;
    }
    double spaceBuffer = -0.02; // -0.05 // positive means search box is inside the movement vector (defined from start to end point) and negative means the search box is bigger than the vector.

    // // X
    // searchSpaceMin(indexCounter) = wyptData.col(1).minCoeff() + spaceBuffer;
    // searchSpaceMax(indexCounter) = wyptData.col(1).maxCoeff() - spaceBuffer;
    // indexCounter++;
    //
    // // Y
    // searchSpaceMin(indexCounter) = wyptData.col(2).minCoeff() + spaceBuffer;
    // searchSpaceMax(indexCounter) = wyptData.col(2).maxCoeff() - spaceBuffer;
    // indexCounter++;
    //
    // // Z
    // searchSpaceMin(indexCounter) = wyptData.col(3).minCoeff() + spaceBuffer;
    // searchSpaceMax(indexCounter) = wyptData.col(3).maxCoeff() - spaceBuffer;


    Eigen::Vector3d middleWaypoint, searchBox;
    searchBox = Eigen::Vector3d(0.05, 0.05, 0.02);
    middleWaypoint = wyptData.rightCols(3).row(1).transpose();
    searchSpaceMin.tail(3) << middleWaypoint - searchBox;
    searchSpaceMax.tail(3) << middleWaypoint + searchBox;


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


void MoveWeight::bottleEigenVector(yarp::os::Bottle& bottle, const Eigen::VectorXd& vecToBottle, const bool encapsulate)
{
    bottle.clear();
    for(int i =0; i<vecToBottle.size(); i++){
        bottle.addDouble(vecToBottle(i));
    }
}

void MoveWeight::checkSolverStatus()
{
    setInitialWaypoints();

    yarp::os::Bottle cmd, response;
    cmd.addString("iteration");
    rpcClientPort.write(cmd,response);
    std::cout << "Got response:\n" << response.toString() << std::endl;
    int currentIter = response.get(0).asInt();
    if (currentIter>0) {

        std::ifstream pathFile;
        pathFile.open((rootLogFilePathPrefix+"/latestLogPath.txt").c_str());
        std::getline(pathFile, rootLogFilePathPrefix);
        pathFile.close();



        testNumber = currentIter + 1;
        optimizationInProgress = true;
        optimumFound = bool(response.get(1).asInt());
        int j=0;
        // int nDims = response.size() - 2;
        // optVariables.resize(nDims);
        for(int i=2; i<=response.size(); i++){
            optVariables(j) = response.get(i).asDouble();
            j++;
        }
        if (optimumFound) {
            std::cout << "\n---------------\nFound optimum!\n---------------\n" << std::endl;
            std::cout <<"[OPTIMAL VARIABLES]\n"<< optVariables.transpose() <<"\n"<< std::endl;
        }else{
            std::cout <<"[NEW TEST VARIABLES]\n"<< optVariables.transpose() <<"\n"<< std::endl;
        }

        /*
        *   Get max variance for weight calcs.
        */
        maxVariance = rightHandTrajectory->getMaxVariance();


        /*
        *   For the trajectory logging
        */
        desiredPosVelAcc_rightHand = Eigen::MatrixXd::Zero(3,3);
        desiredPosVelAcc_rightHand.col(0) << rHandPosStart;
        currentOptWaypoint = optVariables.tail(3);

        rightHandTrajectory->setBoptVariables(optVariables);

        std::ifstream energyCostScalingFactorFile;
        energyCostScalingFactorFile.open((rootLogFilePathPrefix + "/energyCostScalingFactor.txt").c_str());
        if (energyCostScalingFactorFile.is_open()) {
            energyCostScalingFactorFile >> energyCostScalingFactor;
            std::cout << "energyCostScalingFactor: " << energyCostScalingFactor << std::endl;
        }
        energyCostScalingFactorFile.close();

    }else{
        std::ofstream pathFile;
        pathFile.open((rootLogFilePathPrefix+"/latestLogPath.txt").c_str());
        rootLogFilePathPrefix += "/test-" + smlt::currentDateTime() +"/";
        smlt::checkAndCreateDirectory(rootLogFilePathPrefix);
        pathFile << rootLogFilePathPrefix;
        pathFile.close();

    }

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*
*           Logging and file operations
*
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




bool MoveWeight::openLogFiles(const std::string testLogFilePathPrefix)
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

bool MoveWeight::closeLogFiles()
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
