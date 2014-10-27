#include "taskSetTests.h"
#include "iCubTaskGenerator.h"
#include "ISIRCtrlTaskManager.h"
// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"

#include "orcWbiModel.h"

#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <math.h>

using namespace wbiIcub;

void getNominalPosture(const orcWbiModel &model, VectorXd &q);

/* static */ ISIRCtrlTaskManager TaskSet_initialPosHold::getTask(Model& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(model.nbInternalDofs());
    
    // Full posture task
    postureTaskQ = model.getJointPositions();
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 0.01);
    
    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_initialPosZero::getTask(orcWbiModel& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(model.nbInternalDofs());
    
    // Full posture task
    getNominalPosture(model, postureTaskQ);
    //Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(nbInternalDofs);

    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 15, 3, 0.01);
    
    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_initialPosHold_CoMPos_BothHandPos::getTask(orcWbiModel& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();

    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(model.nbInternalDofs());
    
    // Full posture task
    getNominalPosture(model, postureTaskQ);

    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 0.5);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    torso_indices << model.getDOFId("torso_pitch"), model.getDOFId("torso_roll"), model.getDOFId("torso_yaw");
    Eigen::VectorXd torsoTaskPosDes(3);
    torsoTaskPosDes << M_PI / 18, 0, 0;
    iCubPostureTaskGenerator torsoTask = iCubPostureTaskGenerator(tm, "partial_task", torso_indices, orc::FullState::INTERNAL, torsoTaskPosDes, 10, 3, 5.0);

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

/* static */ ISIRCtrlTaskManager TaskSet_fixed_base_walk::getTask(orcWbiModel& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    
    // Full posture task
    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(model.nbInternalDofs());
    //getNominalPosture(model, postureTaskQ);

    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 15, 3, 0.01);
    
    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    torso_indices << model.getDOFId("torso_pitch"), model.getDOFId("torso_roll"), model.getDOFId("torso_yaw");
    Eigen::VectorXd torsoTaskPosDes(3);
    torsoTaskPosDes << M_PI / 18, 0, 0;
    iCubPostureTaskGenerator torsoTask = iCubPostureTaskGenerator(tm, "partial_task", torso_indices, orc::FullState::INTERNAL, torsoTaskPosDes, 10, 3, 1.0);
    
    
    // Left hand, right foot, and right shank cartesian tasks
    Eigen::Vector3d lhpos = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos[0]-=0.2;
    lhpos[2]-=0.2;
    Eigen::Displacementd posLHandDes = Eigen::Displacementd(lhpos);
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 10, 6, 1.0);

    Eigen::Vector3d lhpos2 = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos2[0]-=0.2;
    lhpos2[2]+=0.2;
    Eigen::Displacementd posLHandDes2 = Eigen::Displacementd(lhpos2);
    iCubCartesianTaskGenerator leftHandTask2 = iCubCartesianTaskGenerator(tm, "l_hand_task2", "l_hand", orc::XYZ, posLHandDes2, 10, 6, 1.0);
    leftHandTask2.getTask()->deactivate();

    Eigen::Vector3d rfpos = model.getSegmentPosition(model.getSegmentIndex("r_foot")).getTranslation();
    Eigen::Displacementd posRFootDes = Eigen::Displacementd(rfpos);
    iCubCartesianTaskGenerator rightFootTask = iCubCartesianTaskGenerator(tm, "r_foot_task", "r_foot", orc::XY, posRFootDes, 10, 6, 1.0);


    Eigen::Vector3d rspos = model.getSegmentPosition(model.getSegmentIndex("r_shank")).getTranslation();
    rspos[0]-=0.1;
    Eigen::Displacementd posRShankDes = Eigen::Displacementd(rspos);
    iCubCartesianTaskGenerator rightShankTask = iCubCartesianTaskGenerator(tm, "r_shank_task", "r_shank", orc::XYZ, posRShankDes, 10, 6, 1.0);

    Eigen::Vector3d rspos2 = model.getSegmentPosition(model.getSegmentIndex("r_shank")).getTranslation();
    rspos2[0]-=0.2;
    rspos2[2]+=0.2;
    Eigen::Displacementd posRShankDes2 = Eigen::Displacementd(rspos2);
    iCubCartesianTaskGenerator rightShankTask2 = iCubCartesianTaskGenerator(tm, "r_shank_task2", "r_shank", orc::XYZ, posRShankDes2, 10, 6, 1.0);
    rightShankTask2.getTask()->deactivate();

    //right hand, left foot, and left shank tasks
    Eigen::Vector3d rhpos = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    rhpos[0]-=0.2;
    rhpos[2]+=0.2;
    Eigen::Displacementd posRHandDes = Eigen::Displacementd(rhpos);
    iCubCartesianTaskGenerator rightHandTask = iCubCartesianTaskGenerator(tm, "r_hand_task", "r_hand", orc::XYZ, posRHandDes, 10, 6, 1.0);

    Eigen::Vector3d rhpos2 = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    rhpos2[0]-=0.2;
    rhpos2[2]-=0.2;
    Eigen::Displacementd posRHandDes2 = Eigen::Displacementd(rhpos2);
    iCubCartesianTaskGenerator rightHandTask2 = iCubCartesianTaskGenerator(tm, "r_hand_task2", "r_hand", orc::XYZ, posRHandDes2, 10, 6, 1.0);
    rightHandTask2.getTask()->deactivate();

    Eigen::Vector3d lfpos = model.getSegmentPosition(model.getSegmentIndex("l_foot")).getTranslation();
    Eigen::Displacementd posLFootDes = Eigen::Displacementd(lfpos);
    iCubCartesianTaskGenerator leftFootTask = iCubCartesianTaskGenerator(tm, "l_foot_task", "l_foot", orc::XY, posLFootDes, 10, 6, 1.0);


    Eigen::Vector3d lspos = model.getSegmentPosition(model.getSegmentIndex("l_shank")).getTranslation();   
    lspos[0]-=0.2;    
    lspos[2]+=0.2;
    Eigen::Displacementd posLShankDes = Eigen::Displacementd(lspos);
    iCubCartesianTaskGenerator leftShankTask = iCubCartesianTaskGenerator(tm, "l_shank_task", "l_shank", orc::XYZ, posLShankDes, 10, 6, 1.0);

    Eigen::Vector3d lspos2 = model.getSegmentPosition(model.getSegmentIndex("l_shank")).getTranslation();
    lspos2[2]-=0.1;
    Eigen::Displacementd posLShankDes2 = Eigen::Displacementd(lspos2);
    iCubCartesianTaskGenerator leftShankTask2 = iCubCartesianTaskGenerator(tm, "l_shank_task2", "l_shank", orc::XYZ, posLShankDes2, 10, 6, 1.0);
    leftShankTask2.getTask()->deactivate();
    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_standing::getTask(orcWbiModel& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();

    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(model.nbInternalDofs());

    // Full posture task
    getNominalPosture(model, postureTaskQ);

    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 15, 3, 0.01);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    torso_indices << model.getDOFId("torso_pitch"), model.getDOFId("torso_roll"), model.getDOFId("torso_yaw");
    Eigen::VectorXd torsoTaskPosDes(3);
    torsoTaskPosDes << M_PI / 18, 0, 0;
    iCubPostureTaskGenerator torsoTask = iCubPostureTaskGenerator(tm, "partial_task", torso_indices, orc::FullState::INTERNAL, torsoTaskPosDes, 10, 3, 1);

//    // waist task
//    Eigen::Displacementd posWaist = model.getSegmentPosition(model.getSegmentIndex("chest"));
//    iCubOrientationTaskGenerator waistTask = iCubOrientationTaskGenerator(tm, "waist_task", "chest", posWaist, 36, 12, 0.4);

    // head task
//    Eigen::Displacementd posHead = model.getSegmentPosition(model.getSegmentIndex("head"));
//    iCubOrientationTaskGenerator headTask = iCubOrientationTaskGenerator(tm, "head_task", "head", posHead,10, 6, 0.5);

    // CoM Task
    Eigen::Vector3d posCoM = model.getCoMPosition();
    iCubCoMTaskGenerator comTask = iCubCoMTaskGenerator(tm, "com_task", posCoM, 20, 9, 10.0);

    // Left hand cartesian task
//    Eigen::Displacementd posLHandDes(-0.23, -0.21, 0.3, 1, 0, 0, 0);
    Eigen::Vector3d lhpos = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos[0]-=0.1;
    lhpos[2]+=0.3;
    Eigen::Displacementd posLHandDes = Eigen::Displacementd(lhpos);
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 10, 6,1.0);

//    Eigen::Displacementd posRHandDes(-0.23, 0.21, 0.3, 1, 0, 0, 0);
    Eigen::Vector3d rhpos = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    rhpos[0]-=0.1;
    rhpos[2]+=0.3;
    Eigen::Displacementd posRHandDes = Eigen::Displacementd(rhpos);
    iCubCartesianTaskGenerator rightHandTask = iCubCartesianTaskGenerator(tm, "r_hand_task", "r_hand", orc::XYZ, posRHandDes, 10, 6, 1.0);

    // Foot contact tasks

    double sqrt2on2 = sqrt(2)/2;
    Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(-sqrt2on2,0,-sqrt2on2,0)*Eigen::Rotation3d(0,1,0,0);
    Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(0,sqrt2on2,0,sqrt2on2)*Eigen::Rotation3d(0,1,0,0);

    Eigen::Displacementd lfcontact0 = Eigen::Displacementd(Eigen::Vector3d(-0.039,-0.027,-0.031),rotLZdown);
    Eigen::Displacementd lfcontact1 = Eigen::Displacementd(Eigen::Vector3d(-0.039,0.027,-0.031),rotLZdown);
    Eigen::Displacementd lfcontact2 = Eigen::Displacementd(Eigen::Vector3d(-0.039,0.027,0.099),rotLZdown);
    Eigen::Displacementd lfcontact3 = Eigen::Displacementd(Eigen::Vector3d(-0.039,-0.027,-0.099),rotLZdown);
    iCubContactTaskGenerator leftFootContactTask0 = iCubContactTaskGenerator(tm, "CLF0","l_foot",lfcontact0,1.5,0.0);
    iCubContactTaskGenerator leftFootContactTask1 = iCubContactTaskGenerator(tm, "CLF1","l_foot",lfcontact1,1.5,0.0);
    iCubContactTaskGenerator leftFootContactTask2 = iCubContactTaskGenerator(tm, "CLF2","l_foot",lfcontact2,1.5,0.0);
    iCubContactTaskGenerator leftFootContactTask3 = iCubContactTaskGenerator(tm, "CLF3","l_foot",lfcontact3,1.5,0.0);

    Eigen::Displacementd rfcontact0 = Eigen::Displacementd(Eigen::Vector3d(-0.039,-0.027,0.031),rotRZdown);
    Eigen::Displacementd rfcontact1 = Eigen::Displacementd(Eigen::Vector3d(-0.039,0.027,0.031),rotRZdown);
    Eigen::Displacementd rfcontact2 = Eigen::Displacementd(Eigen::Vector3d(-0.039,0.027,-0.099),rotRZdown);
    Eigen::Displacementd rfcontact3 = Eigen::Displacementd(Eigen::Vector3d(-0.039,-0.027,0.099),rotRZdown);
    iCubContactTaskGenerator rightFootContactTask0 = iCubContactTaskGenerator(tm, "CRF0","r_foot",rfcontact0,1.5,0.0);
    iCubContactTaskGenerator rightFootContactTask1 = iCubContactTaskGenerator(tm, "CRF1","r_foot",rfcontact1,1.5,0.0);
    iCubContactTaskGenerator rightFootContactTask2 = iCubContactTaskGenerator(tm, "CRF2","r_foot",rfcontact2,1.5,0.0);
    iCubContactTaskGenerator rightFootContactTask3 = iCubContactTaskGenerator(tm, "CRF3","r_foot",rfcontact3,1.5,0.0);
    return tm;
}

void getNominalPosture(const orcWbiModel &model, VectorXd &q)
{
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
