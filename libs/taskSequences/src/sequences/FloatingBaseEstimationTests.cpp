#include <taskSequences/sequences/FloatingBaseEstimationTests.h>
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void FloatingBaseEstimationTests::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        Eigen::VectorXd q_init = model.getJointPositions();
        // std::cout << "q init: " << q_init << std::endl;
        taskManagers["tmFull"] = new ocra::FullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, 1.0, q_init);
    }

    void FloatingBaseEstimationTests::doUpdate(double time, ocra::Model& state, void** args)
    {
    }
// }
