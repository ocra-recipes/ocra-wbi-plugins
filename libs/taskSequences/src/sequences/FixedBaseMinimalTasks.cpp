#include <taskSequences/sequences/FixedBaseMinimalTasks.h>
// FixedBaseMinimalTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void FixedBaseMinimalTasks::doInit(wocra::wOcraController& ctrl, ocra::Model& model)
    {

        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getHomePosture(model, q_full);


        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 10.0, 2*sqrt(10.0), 1.0, q_full);
    }

    void FixedBaseMinimalTasks::doUpdate(double time, ocra::Model& state, void** args)
    {
    }

// }
