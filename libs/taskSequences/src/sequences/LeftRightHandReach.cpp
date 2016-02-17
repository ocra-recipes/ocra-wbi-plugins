#include <taskSequences/sequences/LeftRightHandReach.h>
// LeftRightHandReach
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void LeftRightHandReach::doInit(wocra::wOcraController& ctrl, ocra::Model& model)
    {
        ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);

        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, 0.01, nominal_q);

        // Partial (torso) posture task

        Eigen::VectorXi torso_indices(3);
        Eigen::VectorXd torsoTaskPosDes(3);
        torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
        torsoTaskPosDes << M_PI / 18, 0, 0;
        taskManagers["tmPartialTorso"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 10.0, 3.0, 5.0, torsoTaskPosDes);


        // CoM Task
        Eigen::Vector3d posCoM = model.getCoMPosition();
        taskManagers["tmCoM"] = new wocra::wOcraCoMTaskManager(ctrl, model, "CoMTask", ocra::XYZ, 10.0, 3.0, 10.0, posCoM);

        // Left hand cartesian task
        Eigen::Vector3d posLHandDes(-0.3, -0.2, 0.15);
        taskManagers["tmSegCartHandLeft"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 10.0, 3.0, 100.0, posLHandDes);


        // Right hand cartesian task
        Eigen::Vector3d posRHandDes(-0.15, 0.2, -0.1);
        taskManagers["tmSegCartHandRight"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, 10.0, 3.0, 100.0, posRHandDes);


    }

    void LeftRightHandReach::doUpdate(double time, ocra::Model& state, void** args)
    {
    }
// }
