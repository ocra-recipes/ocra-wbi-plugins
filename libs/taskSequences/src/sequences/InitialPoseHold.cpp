#include <taskSequences/sequences/InitialPoseHold.h>
// InitialPoseHold


// namespace sequence{
    void InitialPoseHold::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        Eigen::VectorXd q_init = model.getJointPositions();
        std::cout << "q init: " << q_init << std::endl;
        taskManagers["tmFull"] = std::make_shared<ocra::FullPostureTaskManager>(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, 1.0, q_init);
    }

    void InitialPoseHold::doUpdate(double time, ocra::Model& state, void** args)
    {
    }
// }
