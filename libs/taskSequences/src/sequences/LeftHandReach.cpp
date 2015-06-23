#include <taskSequences/sequences/LeftHandReach.h>
// LeftHandReach
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void LeftHandReach::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
    {
        ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);

        tmFull = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, 0.001, nominal_q);

        // Left hand cartesian task
        Eigen::Vector3d posLHandDes(-0.3, -0.1, 0.3);
        tmSegCartHandLeft = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 10.0, 3.0, 100.0, posLHandDes);
    }

    void LeftHandReach::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
    }
// }
