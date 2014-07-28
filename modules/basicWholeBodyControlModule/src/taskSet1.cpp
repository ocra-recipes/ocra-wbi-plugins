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
    
    Eigen::VectorXd postureTaskQ = Eigen::VectorXd::Zero(nbInternalDofs);
    //iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,const std::string& taskName,int state, Eigen::VectorXd& target, double stiffness, double damping, double weight)
    iCubPostureTaskGenerator postureTask = iCubPostureTaskGenerator(tm, "full_task", orc::FullState::INTERNAL, postureTaskQ, 10, 3, 0.01);

    Eigen::Vector3d posCoM = model.getCoMPosition();
    //iCubCoMTaskGenerator(ISIRCtrlTaskManager& tManager,const std::string& taskName, Eigen::Displacementd target, double stiffness, double damping, double weight);
    iCubCoMTaskGenerator comTask = iCubCoMTaskGenerator(tm,"com_task",posCoM,5,1,100);

    Eigen::Displacementd posLHandDes(0.3, -0.3, 0.2, 1, 0, 0, 0);
    // iCubCartesianTaskGenerator(ISIRCtrlTaskManager& tManager,const std::string& taskName,const std::string& segmentName,orc::ECartesianDof axes, Eigen::Displacementd target, double stiffness, double damping, double weight)
    iCubCartesianTaskGenerator leftHandTask = iCubCartesianTaskGenerator(tm, "l_hand_task", "l_hand", orc::XYZ, posLHandDes, 5, 1, 1.0);

    return tm;
}

