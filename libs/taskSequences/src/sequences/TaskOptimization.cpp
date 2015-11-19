#include <taskSequences/sequences/TaskOptimization.h>
#include <ocraWbiPlugins/ocraWbiModel.h>


TaskOptimization::~TaskOptimization()
{
    optVarsPortOut.close();
    costPortOut.close();
    optVarsPortIn.close();
}

void TaskOptimization::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    // Task Coeffs
    double Kp_posture = 5.0;
    double Kd_posture = 2.0 * sqrt(Kp_posture);
    double wFullPosture = 0.0001;
    double wPartialPosture = 0.1;

    double Kp_hand = 10.0;
    double Kd_hand = 2.0 *sqrt(Kp_hand);
    double wLeftHandCartTask = 1.0;
    double wRightHandCartTask = 1.0;

    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getHomePosture(model, nominal_q);
    taskManagers["fullPostureTask"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp_posture, Kd_posture, wFullPosture, nominal_q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");

    torsoTaskPosDes << 0.0, 0.0, 0.0;
    taskManagers["torsoPostureTask"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "torsoPostureTask", ocra::FullState::INTERNAL, torso_indices, Kp_posture, Kd_posture, wPartialPosture, torsoTaskPosDes);



    // taskManagers["leftHandCartesianTask"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandCartTask);


    taskManagers["rightHandCartesianTask"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, wRightHandCartTask);


    optVarsPortOut.open("/opt/vars:o");
    costPortOut.open("/opt/cost:o");
    optVarsPortIn.open("/opt/vars:i");
}



void TaskOptimization::doUpdate(double time, wocra::wOcraModel& state, void** args)
{




}
