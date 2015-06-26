#include <taskSequences/sequences/Debug.h>
// FixedBaseMinimalTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void Debug::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
    {

        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, q_full);


        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 10.0, 2*sqrt(10.0), 1.0, q_full);
    }

    void Debug::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
    }

// }
