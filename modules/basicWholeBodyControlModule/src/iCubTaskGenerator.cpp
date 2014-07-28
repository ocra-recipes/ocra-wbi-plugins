
#include "iCubTaskGenerator.h"

//Cartesian task generator
iCubCartesianTaskGenerator::iCubCartesianTaskGenerator(ISIRCtrlTaskManager& tManager,const std::string& taskName,const std::string& segmentName,orc::ECartesianDof axes, Eigen::Displacementd target, double stiffness, double damping, double weight)
    :model(tManager.getModel())
    ,taskManager(tManager)
{
    SF = new orc::SegmentFrame("frame.SFrame"+segmentName, model, segmentName, Eigen::Displacementd());
    TF = new orc::TargetFrame("frame.TFrame"+segmentName, model);
    feat = new orc::PositionFeature("frame"+segmentName, *SF, axes);
    featDes = new orc::PositionFeature("frame.Des"+segmentName, *TF, axes);
    posdes = target;
    TF->setPosition(posdes);
    TF->setVelocity(Eigen::Twistd());
    TF->setAcceleration(Eigen::Twistd());
    task = &(taskManager.getCtrl().createISIRTask(taskName, *feat, *featDes));
    task->initAsAccelerationTask();
    taskManager.addTask2TaskList(task);

    task->activateAsObjective();
    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);

}

iCubCartesianTaskGenerator::~iCubCartesianTaskGenerator()
{

}

orc::PositionFeature* iCubCartesianTaskGenerator::getFeature()
{
    return feat;
}
orc::PositionFeature* iCubCartesianTaskGenerator::getFeatureDes()
{
    return featDes;
}
orcisir::ISIRTask* iCubCartesianTaskGenerator::getTask()
{
    return task;
}

// Posture task generator
iCubPostureTaskGenerator::iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,
													const std::string& taskName,
													int state, 
													Eigen::VectorXd& target, 
													double stiffness, 
													double damping, 
													double weight)
    
													:model(tManager.getModel())
													,taskManager(tManager)
													,posdes(target)
{
    FMS 	= new orc::FullModelState	(taskName+".FModelState", model, state); //axes=INTERNAL, FREE_FLYER
    FTS 	= new orc::FullTargetState	(taskName+".FTargetState", model, state);
    feat 	= new orc::FullStateFeature	(taskName+".feat", *FMS);
    featDes = new orc::FullStateFeature	(taskName+".featDes", *FTS);

    FTS->set_q(posdes);

    task = &(taskManager.getCtrl().createISIRTask(taskName, *feat, *featDes));
    task->initAsAccelerationTask();
    taskManager.addTask2TaskList(task);

    task->activateAsObjective();
    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);

}


iCubPostureTaskGenerator::iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,
													const std::string& taskName,
													Eigen::VectorXi& selected_dof_indices,
													int state, 
													Eigen::VectorXd& target, 
													double stiffness, 
													double damping, 
													double weight)
							
													:model(tManager.getModel())
													,taskManager(tManager)
													,posdes(target)
{

    PMS 		= new orcisir::PartialModelState   (taskName+".PModelState", model,  selected_dof_indices, state); 
    PTS 		= new orcisir::PartialTargetState  (taskName+".PTargetState", model, selected_dof_indices, state);
    p_feat 		= new orcisir::PartialStateFeature (taskName+".P_feat", *PMS);
    p_featDes 	= new orcisir::PartialStateFeature (taskName+".P_featDes", *PTS);

    PTS->set_q(posdes);
	
    task = &(taskManager.getCtrl().createISIRTask(taskName, *p_feat, *p_featDes));
    std::cout << posdes << "\n\n\n";
    task->initAsAccelerationTask();
    taskManager.addTask2TaskList(task);

    task->activateAsObjective();
    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);
    

}

iCubPostureTaskGenerator::~iCubPostureTaskGenerator()
{

}

orc::FullStateFeature* iCubPostureTaskGenerator::getFeature()
{
    return feat;
}
orc::FullStateFeature* iCubPostureTaskGenerator::getFeatureDes()
{
    return featDes;
}
orcisir::ISIRTask* iCubPostureTaskGenerator::getTask()
{
    return task;
}

//================ PARTIAL STATE ==============================================================================//

