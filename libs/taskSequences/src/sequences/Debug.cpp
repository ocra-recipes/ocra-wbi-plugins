#include <taskSequences/sequences/Debug.h>
// FixedBaseMinimalTasks


// namespace sequence{
    void Debug::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {

        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, q_full);


        taskManagers["tmFull"] = std::make_shared<ocra::FullPostureTaskManager>(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 150.0, 2*sqrt(150.0), 1.0, q_full);
    }

    void Debug::doUpdate(double time, ocra::Model& state, void** args)
    {
    }

// }
