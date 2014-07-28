
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
iCubPostureTaskGenerator::iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,const std::string& taskName,int state, Eigen::VectorXd& target, double stiffness, double damping, double weight)
    :model(tManager.getModel())
    ,taskManager(tManager)
{
    FMS = new orc::FullModelState("torqueTask.FModelState", model, state); //axes=INTERNAL, FREE_FLYER
    FTS = new orc::FullTargetState("torqueTask.FTargetState", model, state);

    feat = new orc::FullStateFeature("torqueTask", *FMS);
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);
    posdes = target;
    FTS->set_q(posdes);

    task = &(taskManager.getCtrl().createISIRTask(taskName, *feat, *featDes));
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
