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

    std::cout << "TEST\n";
    std::cout << model.getDOFId("l_shoulder_roll") << std::endl;
    std::cout << model.getDOFId("r_shoulder_roll") << std::endl;
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
//    postureTaskQ(13,0) = -M_PI / 4; //r_shoulder_pitch
//    postureTaskQ(14,0) = M_PI / 4;  //r_shoulder_roll
//    postureTaskQ(16,0) = M_PI / 2;  //r_elbow
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 1);

    // Partial (torso) posture task
//    Eigen::VectorXi torso_indices(3);
//    torso_indices << 0, 1, 2;
//    Eigen::VectorXd torsoTaskPosDes = Eigen::VectorXd::Zero(3);
//    iCubPostureTaskGenerator torsoTask = iCubPostureTaskGenerator(tm, "partial_task", torso_indices, orc::FullState::INTERNAL, torsoTaskPosDes, 10, 3, 1.0);

    Eigen::Vector3d posCoM = model.getCoMPosition();
    iCubCoMTaskGenerator comTask = iCubCoMTaskGenerator(tm, "com_task", posCoM, 10, 3, 10.0);
    // Left hand cartesian task
//    Eigen::Displacementd posLHandDes(-0.23, -0.21, 0.3, 1, 0, 0, 0);
    Eigen::Vector3d lhpos = model.getSegmentPosition(model.getSegmentIndex("l_hand")).getTranslation();
    lhpos[0]+=0.02;
    lhpos[2]+=0.5;
    Eigen::Displacementd posLHandDes = Eigen::Displacementd(lhpos);
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 20, 3, 2.0);

//    Eigen::Displacementd posRHandDes(-0.23, 0.21, 0.3, 1, 0, 0, 0);
    Eigen::Vector3d rhpos = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    rhpos[0]+=0.02;
    rhpos[2]+=0.5;
    Eigen::Displacementd posRHandDes = Eigen::Displacementd(rhpos);
    iCubCartesianTaskGenerator rightHandTask = iCubCartesianTaskGenerator(tm, "r_hand_task", "r_hand", orc::XYZ, posRHandDes, 20, 3, 2.0);

    return tm;
}

/* static */ ISIRCtrlTaskManager TaskSet_initialPosHold_leftHandPos::getTask(Model& model, orcisir::ISIRController& ctrl)
{
    ISIRCtrlTaskManager tm = ISIRCtrlTaskManager(model, ctrl);
    int nbInternalDofs = model.nbInternalDofs();
    
    // Full posture task
    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(nbInternalDofs);
    postureTaskQ(13,0) = -M_PI / 4; //r_shoulder_pitch
    postureTaskQ(14,0) = M_PI / 4;  //r_shoulder_roll
    postureTaskQ(16,0) = M_PI / 2;  //r_elbow
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 0.01);
    
    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    torso_indices << 0, 1, 2;
    Eigen::VectorXd torsoTaskPosDes = Eigen::VectorXd::Zero(3);
    iCubPostureTaskGenerator torsoTask = iCubPostureTaskGenerator(tm, "partial_task", torso_indices, orc::FullState::INTERNAL, torsoTaskPosDes, 10, 3, 1.0);
    
    
    // Left hand cartesian task
    Eigen::Displacementd posLHandDes(0.3, -0.3, 0.2, 1, 0, 0, 0);
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 5, 1, 1.0);

    return tm;
}

