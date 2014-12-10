#include "taskManagerCollection.h"
#include "orcWbiModel.h"

#ifndef PI
#define PI 3.1415926
#endif

void getNominalPosture(orc::Model &model, VectorXd &q);

// TaskCollection_InitialPoseHold
void TaskCollection_InitialPoseHold::doInit(orcisir::ISIRController& ctrl, orc::Model& model)
{
    Eigen::VectorXd q_init = model.getJointPositions();
    std::cout << "q init: " << q_init << std::endl;
    tmFull = new orcisir::ISIRFullPostureTaskManager(ctrl, model, "fullPostureTask", orc::FullState::INTERNAL, 20.0, 3.0, 1.0, q_init);
}

void TaskCollection_InitialPoseHold::doUpdate(double time, orc::Model& state, void** args)
{
}

// TaskCollection_NominalPoseHold
void TaskCollection_NominalPose::doInit(orcisir::ISIRController& ctrl, orc::Model& model)
{
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);

    tmFull = new orcisir::ISIRFullPostureTaskManager(ctrl, model, "fullPostureTask", orc::FullState::INTERNAL, 20.0, 5.0, 1, nominal_q);
}

void TaskCollection_NominalPose::doUpdate(double time, orc::Model& state, void** args)
{
    std::cout << "Time: " << time << "[s], Posture error total: " << tmFull->getTaskErrorNorm() << std::endl;

    Eigen::VectorXd taskError = tmFull->getTaskError();
    for (int i = 0; i < taskError.size(); i++)
        std::cout << "Joint " << i << ", Error : " << taskError(i) << std::endl; 

    std::cout << std::endl << std::endl;
}

// TaskCollection_LeftHandReach
void TaskCollection_LeftHandReach::doInit(orcisir::ISIRController& ctrl, orc::Model& model)
{
    orcWbiModel& wbiModel = dynamic_cast<orcWbiModel&>(model);
    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);

    tmFull = new orcisir::ISIRFullPostureTaskManager(ctrl, model, "fullPostureTask", orc::FullState::INTERNAL, 20.0, 3.0, 0.001, nominal_q);

    // Left hand cartesian task
    Eigen::Vector3d posLHandDes(-0.3, -0.1, 0.3);
    tmSegCartHandLeft = new orcisir::ISIRSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", orc::XYZ, 10.0, 3.0, 100.0, posLHandDes);
}

void TaskCollection_LeftHandReach::doUpdate(double time, orc::Model& state, void** args)
{
}


// TaskCollection_LeftRightHandReach
void TaskCollection_LeftRightHandReach::doInit(orcisir::ISIRController& ctrl, orc::Model& model)
{
    orcWbiModel& wbiModel = dynamic_cast<orcWbiModel&>(model);
    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);

    tmFull = new orcisir::ISIRFullPostureTaskManager(ctrl, model, "fullPostureTask", orc::FullState::INTERNAL, 20.0, 3.0, 0.01, nominal_q);

     // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDOFId("torso_pitch"), wbiModel.getDOFId("torso_roll"), wbiModel.getDOFId("torso_yaw");
    torsoTaskPosDes << M_PI / 18, 0, 0;
    tmPartialTorso = new orcisir::ISIRPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", orc::FullState::INTERNAL, torso_indices, 10.0, 3.0, 5.0, torsoTaskPosDes);

    // CoM Task
    Eigen::Vector3d posCoM = model.getCoMPosition();
    tmCoM = new orcisir::ISIRCoMTaskManager(ctrl, model, "CoMTask", 10.0, 3.0, 10.0, posCoM);

    // Left hand cartesian task
    Eigen::Vector3d posLHandDes(-0.3, -0.2, 0.15);
    tmSegCartHandLeft = new orcisir::ISIRSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", orc::XYZ, 10.0, 3.0, 100.0, posLHandDes);

    // Right hand cartesian task
    Eigen::Vector3d posRHandDes(-0.15, 0.2, -0.1);
    tmSegCartHandRight = new orcisir::ISIRSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", orc::XYZ, 10.0, 3.0, 100.0, posRHandDes);

    
}

void TaskCollection_LeftRightHandReach::doUpdate(double time, orc::Model& state, void** args)
{
}

void getNominalPosture(orc::Model& orcmodel, VectorXd &q)
{
    orcWbiModel& model = dynamic_cast<orcWbiModel&>(orcmodel);
    
    q[model.getDOFId("torso_pitch")] = M_PI / 18;
    q[model.getDOFId("r_elbow")] = M_PI / 4;
    q[model.getDOFId("l_elbow")] = M_PI / 4;
    q[model.getDOFId("l_shoulder_roll")] = M_PI / 6;
    q[model.getDOFId("r_shoulder_roll")] = M_PI / 6;
    q[model.getDOFId("l_shoulder_pitch")] = -M_PI / 6;
    q[model.getDOFId("r_shoulder_pitch")] = -M_PI / 6;
    q[model.getDOFId("l_hip_pitch")] = M_PI / 6;
    q[model.getDOFId("r_hip_pitch")] = M_PI / 6;
    q[model.getDOFId("l_hip_roll")] = M_PI / 18;
    q[model.getDOFId("r_hip_roll")] = M_PI / 18;
    q[model.getDOFId("l_knee")] = -M_PI / 4;
    q[model.getDOFId("r_knee")] = -M_PI / 4;
}
