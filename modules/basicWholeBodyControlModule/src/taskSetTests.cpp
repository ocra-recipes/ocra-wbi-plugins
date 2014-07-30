#include "taskSetTests.h"
#include "iCubTaskGenerator.h"
#include "ISIRCtrlTaskManager.h"
// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"

#include "orcWbiModel.h"

#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>

using namespace wbiIcub;


/* static */ ISIRCtrlTaskManager TaskSet_initialPosHold::getTask(Model& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    
    // Full posture task
    Eigen::VectorXd postureTaskQ = model.getJointPositions();
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 0.01);
    
    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_initialPosZero::getTask(orcWbiModel& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    
    // Full posture task
    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(nbInternalDofs);

    postureTaskQ[model.getDOFId("l_shoulder_roll")] = M_PI / 10;  //l_shoulder_roll 
    postureTaskQ[model.getDOFId("r_shoulder_roll")] = M_PI / 10;  //r_shoulder_roll 
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 0.01);
    
    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_initialPosHold_CoMPos_BothHandPos::getTask(Model& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();

    // Full posture task
    Eigen::VectorXd postureTaskQ = model.getJointPositions();
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 1);

    // CoM Task
    Eigen::Vector3d posCoM = model.getCoMPosition();
    iCubCoMTaskGenerator comTask = iCubCoMTaskGenerator(tm, "com_task", posCoM, 10, 3, 10.0);

    // Left hand cartesian task
//    Eigen::Displacementd posLHandDes(-0.23, -0.21, 0.3, 1, 0, 0, 0);
    Eigen::Vector3d lhpos = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos[0]-=0.1;
    lhpos[2]+=0.5;
    Eigen::Displacementd posLHandDes = Eigen::Displacementd(lhpos);
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 20, 3, 2.0);

//    Eigen::Displacementd posRHandDes(-0.23, 0.21, 0.3, 1, 0, 0, 0);
    Eigen::Vector3d rhpos = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    rhpos[0]-=0.3;
    rhpos[2]+=0.3;
    Eigen::Displacementd posRHandDes = Eigen::Displacementd(rhpos);
    iCubCartesianTaskGenerator rightHandTask = iCubCartesianTaskGenerator(tm, "r_hand_task", "r_hand", orc::XYZ, posRHandDes, 20, 3, 2.0);

    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_initialPosHold_leftHandPos::getTask(orcWbiModel& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    
    // Full posture task
    Eigen::VectorXd postureTaskQ = model.getJointPositions();
    postureTaskQ(model.getDOFId("r_shoulder_pitch")) = -M_PI / 4;
    postureTaskQ(model.getDOFId("r_shoulder_roll")) = M_PI / 4;
    postureTaskQ(model.getDOFId("r_elbow")) = M_PI / 2;

    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 15, 3, 0.01);
    
    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    torso_indices << model.getDOFId("torso_pitch"), model.getDOFId("torso_roll"), model.getDOFId("torso_yaw");
    Eigen::VectorXd torsoTaskPosDes = Eigen::VectorXd::Zero(3);
    iCubPostureTaskGenerator torsoTask = iCubPostureTaskGenerator(tm, "partial_task", torso_indices, orc::FullState::INTERNAL, torsoTaskPosDes, 15, 3, 1.0);
    
    // Left hand cartesian task
//    Eigen::Displacementd posLHandDes(0.3, -0.3, 0.2, 1, 0, 0, 0);
    Eigen::Vector3d lhpos = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos[0]-=0.2;
    lhpos[2]-=0.2;
    Eigen::Displacementd posLHandDes = Eigen::Displacementd(lhpos);
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 10, 3, 1.0);

    Eigen::Vector3d lhpos2 = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos2[0]-=0.2;
    lhpos2[2]+=0.2;
    Eigen::Displacementd posLHandDes2 = Eigen::Displacementd(lhpos2);
    iCubCartesianTaskGenerator leftHandTask2 = iCubCartesianTaskGenerator(tm, "l_hand_task2", "l_hand", orc::XYZ, posLHandDes2, 10, 3, 1.0);
    leftHandTask2.getTask()->deactivate();

    Eigen::Vector3d rfpos = model.getSegmentPosition(model.getSegmentIndex("r_foot")).getTranslation();
    Eigen::Displacementd posRFootDes = Eigen::Displacementd(rfpos);
    iCubCartesianTaskGenerator rightFootTask = iCubCartesianTaskGenerator(tm, "r_foot_task", "r_foot", orc::XY, posRFootDes, 10, 3, 1.0);


    Eigen::Vector3d rspos = model.getSegmentPosition(model.getSegmentIndex("r_shank")).getTranslation();
    rspos[0]-=0.2;
    rspos[2]+=0.2;
    Eigen::Displacementd posRShankDes = Eigen::Displacementd(rspos);
    iCubCartesianTaskGenerator rightShankTask = iCubCartesianTaskGenerator(tm, "r_shank_task", "r_shank", orc::XYZ, posRShankDes, 10, 3, 1.0);

    Eigen::Vector3d rspos2 = model.getSegmentPosition(model.getSegmentIndex("r_shank")).getTranslation();
    rspos2[0]-=0.1;
    Eigen::Displacementd posRShankDes2 = Eigen::Displacementd(rspos2);
    iCubCartesianTaskGenerator rightShankTask2 = iCubCartesianTaskGenerator(tm, "r_shank_task2", "r_shank", orc::XYZ, posRShankDes2, 10, 3, 1.0);
    rightShankTask2.getTask()->deactivate();

    return tm;
}

