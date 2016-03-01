#include <taskSequences/sequences/LeftHandReach.h>
// LeftHandReach


// namespace sequence{
    void LeftHandReach::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        // ocraWbiModel& model = dynamic_cast<ocraWbiModel&>(model);
        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);

        taskManagers["tmFull"] = std::make_shared<ocra::FullPostureTaskManager>(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, 0.001, nominal_q);

        // Left hand cartesian task
        Eigen::Vector3d posLHandDes(-0.3, -0.1, 0.3);
        taskManagers["tmSegCartHandLeft"] = std::make_shared<ocra::SegCartesianTaskManager>(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 10.0, 3.0, 100.0, posLHandDes);
    }

    void LeftHandReach::doUpdate(double time, ocra::Model& state, void** args)
    {
    }
// }
