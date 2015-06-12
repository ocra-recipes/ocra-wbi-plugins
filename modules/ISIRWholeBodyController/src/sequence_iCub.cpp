#include "ISIRWholeBodyController/sequence_iCub.h"
#include <ISIRWholeBodyController/ocraWbiModel.h>

#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"

#include <cmath>

#ifndef PI
#define PI 3.1415926

#endif

sequence_iCub_01_Standing::ScenarioICub_01_Standing() : wocra::wOcraTaskManagerCollectionBase()
{
}

sequence_iCub_01_Standing::~ScenarioICub_01_Standing()
{
}

void sequence_iCub_01_Standing::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    // Initialise full posture task
    // Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
    Eigen::VectorXd q_full = model.getJointPositions();
    
    std::cout << "\n\n q_full = \n"<< q_full <<" \n\n";    

    // q_full[model.getDofIndex("l_elbow")]      = PI/8.0;
    // q_full[model.getDofIndex("r_elbow")]      = PI/8.0;
    // q_full[model.getDofIndex("l_knee")]             = -0.05;
    // q_full[model.getDofIndex("r_knee")]             = -0.05;
    // q_full[model.getDofIndex("l_ankle_pitch")]      = -0.05;
    // q_full[model.getDofIndex("r_ankle_pitch")]      = -0.05;
    // q_full[model.getDofIndex("l_shoulder_roll")]    = PI/8.0;
    // q_full[model.getDofIndex("r_shoulder_roll")]    = PI/8.0;
    
    std::cout << "\n\n q_full = \n"<< q_full <<" \n\n";
    
    std::cout << "\n\n tmFull \n\n";
    taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 10.0, 2*sqrt(10.0), 0.0001, q_full);



/* This part needs to be fixed... Not sure why but the orientation 
    // Initialise waist pose
    
    // std::cout << "\n\n tmSegPoseWaist \n\n";
    // Eigen::Displacementd desiredWaistPose = Eigen::Displacementd(0.0,0.0,0.59,1.0,0.0,0.0,0.0);
    // taskManagers["tmSegPoseWaist"] = new wocra::wOcraSegPoseTaskManager(ctrl, model, "waistPoseTask", "root_link", ocra::XYZ, 36.0, 2*sqrt(36.0), 1.0, desiredWaistPose);
    */
    Eigen::Vector3d desiredWaistPosition, XYZdisp;
    desiredWaistPosition = model.getSegmentPosition(model.getSegmentIndex("root_link")).getTranslation();
    
    

    // XYZdisp << -0.03, 0.0, 0.0;
    // desiredWaistPosition = desiredWaistPosition + XYZdisp;
    taskManagers["tmSegPoseWaist"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "waistPoseTask", "root_link", ocra::XYZ, 36.0, 2*sqrt(36.0), 1.0, desiredWaistPosition);








    // Initialise partial posture task

    
    // Eigen::VectorXi sdofs(3);
    // sdofs << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");
    // Eigen::VectorXd zero = Eigen::VectorXd::Zero(3);
    // std::cout << "\n\n tmPartialBack \n\n";
    // taskManagers["tmPartialBack"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureBackTask", ocra::FullState::INTERNAL, sdofs, 16.0, 2*sqrt(16.0), 0.001, zero);

    double mu_sys = 0.5;
    double margin = 0.0;
    double sqrt2on2 = sqrt(2.0)/2.0;
    
    Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(sqrt2on2, 0.0, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
    Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, 0.0, -sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
    

    // Initialise left foot contacts
    Eigen::Displacementd LFContacts[4];
    LFContacts[0] = Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotLZdown);
    LFContacts[1] = Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotLZdown);
    LFContacts[2] = Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotLZdown);
    LFContacts[3] = Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotLZdown);
    std::cout << "\n\n tmFootContactLeft \n\n";
    taskManagers["tmFootContactLeft"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_sole", LFContacts, 4, mu_sys, margin);

    // Initailise right foot contacts
    Eigen::Displacementd RFContacts[4];
    RFContacts[0] = Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotRZdown);
    RFContacts[1] = Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotRZdown);
    RFContacts[2] = Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotRZdown);
    RFContacts[3] = Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotRZdown);
    std::cout << "\n\n tmFootContactRight \n\n";
    taskManagers["tmFootContactRight"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "RightFootContactTask", "r_sole", RFContacts, 4, mu_sys, margin);
    std::cout << "\n\n Fully Initialized \n\n";
    
}

void sequence_iCub_01_Standing::doUpdate(double time, wocra::wOcraModel& state, void** args)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



sequence_iCub_02_VariableWeightHandTasks::ScenarioICub_02_VariableWeightHandTasks() : wocra::wOcraTaskManagerCollectionBase()
{
}

sequence_iCub_02_VariableWeightHandTasks::~ScenarioICub_02_VariableWeightHandTasks()
{
}

void sequence_iCub_02_VariableWeightHandTasks::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
    
    // Task Coeffs
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);

    double Kp_hand = 40.0;
    double Kd_hand = 8.0 ;//* sqrt(Kp_hand);
    double wFullPosture = 0.0001;
    double wPartialPosture = 0.1;
    Eigen::Vector3d wLeftHandTask = Eigen::Vector3d::Ones();

    // Full posture task
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    q[model.getDofIndex("torso_pitch")] = M_PI / 18;
    q[model.getDofIndex("r_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("r_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("l_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("r_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("l_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("r_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("l_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("r_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("l_knee")] = -M_PI / 6;
    q[model.getDofIndex("r_knee")] = -M_PI / 6;

    taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << 0, -10.0*(M_PI / 180.0), 40.0*(M_PI / 180.0);
    // torsoTaskPosDes << 0.0, 0.0, 0.0;
    taskManagers["tmPartialTorso"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 6., 2.0 * sqrt(6.), wPartialPosture, torsoTaskPosDes);
    
    
    lHandIndex = model.getSegmentIndex("l_hand");
    
           
    // start and end positions
    Eigen::Vector3d startingPos = model.getSegmentPosition(lHandIndex).getTranslation();
      
    
    // multiple position waypoints
    Eigen::MatrixXd waypoints(3,5);
    Eigen::MatrixXd squareDisplacement(3,5);
    waypoints << startingPos, startingPos, startingPos, startingPos, startingPos;
    squareDisplacement << 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.2, 0.2, 0.0, 0.0,
                          0.0, 0.0, 0.2, 0.2, 0.0;
    waypoints += squareDisplacement;

    leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(waypoints);
    leftHandTrajectory->generateTrajectory(3.0); // set a 4 second duration


    tmLeftHand = new wocra::wOcraVariableWeightsTaskManager(ctrl, model, "leftHandTask", "l_hand", Kp_hand, Kd_hand, wLeftHandTask, startingPos);

    Eigen::Vector3d testWeights;
    testWeights << 0.0, 1.0, 0.0;
    tmLeftHand->setWeight(testWeights);

    
}

void sequence_iCub_02_VariableWeightHandTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
{
    Eigen::MatrixXd desiredPosVelAcc = leftHandTrajectory->getDesiredValues(time);
    tmLeftHand->setState(desiredPosVelAcc.col(0));
}
