#include <taskSequences/sequences/FloatingBaseEstimationTests.h>
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void FloatingBaseEstimationTests::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
    {
        Eigen::VectorXd q_init = model.getJointPositions();
        std::cout << "q init: " << q_init << std::endl;
        tmFull = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, 1.0, q_init);
    }

    void FloatingBaseEstimationTests::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
    }
// }
