#include "taskSet1.h"
#include "iCubTaskGenerator.h"
#include "ISIRCtrlTaskManager.h"
// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"


/* static */ ISIRCtrlTaskManager TaskSet1::getTask(Model& model, orcisir::ISIRController& ctrl)
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
    
    
    
	// CoM posture task
    Eigen::Vector3d posCoM = model.getCoMPosition();
    iCubCoMTaskGenerator comTask = iCubCoMTaskGenerator(tm, "com_task", posCoM, 5, 1, 1.0);
    
    
    // Left hand cartesian task
    Eigen::Displacementd posLHandDes(0.3, -0.3, 0.2, 1, 0, 0, 0);
	iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 5, 1, 1.0);

    return tm;
}

