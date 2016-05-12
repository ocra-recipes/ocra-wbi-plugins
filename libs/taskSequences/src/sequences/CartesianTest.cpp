#include <taskSequences/sequences/CartesianTest.h>



// namespace sequence{
    void CartesianTest::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        // Task Coeffs
        double Kp = 10.0;
        double Kd = 2.0 * sqrt(Kp);
        double wFullPosture = 0.001;
        double wPartialPosture = 0.01;
        double wLeftHandTask = 1.0;

        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);
        taskManagers["tmFull"] = std::make_shared<ocra::FullPostureTaskManager>(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

        // Partial (torso) posture task
        Eigen::VectorXi torso_indices(3);
        Eigen::VectorXd torsoTaskPosDes(3);
        torso_indices << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");
        torsoTaskPosDes << M_PI / 18, 0, 0;
        taskManagers["tmPartialTorso"] = std::make_shared<ocra::PartialPostureTaskManager>(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, wPartialPosture, torsoTaskPosDes);

        lHandIndex = model.getSegmentIndex("l_hand");

        Eigen::Vector3d YZ_disp;
        YZ_disp << 0.0, 0.2, 0.2;


        Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);

        // start and end positions
        Eigen::Vector3d startingPos = startingDispd.getTranslation();
        desiredPos  = startingPos+ YZ_disp;


        // Left hand cartesian task
        taskManagers["tmLeftHandCart"] = std::make_shared<ocra::SegCartesianTaskManager>(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp, Kd, wLeftHandTask, desiredPos);

    }

    void CartesianTest::doUpdate(double time, ocra::Model& state, void** args)
    {
        ocra::SegCartesianTaskManager*   tmp_tmLeftHandCart = dynamic_cast<ocra::SegCartesianTaskManager*>(taskManagers["tmLeftHandCart"].get());
        // tmLeftHandCart->setPosition(desiredPos);
        std::cout << "\n---\nDesired position: " << desiredPos.transpose() << std::endl;
        std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
        std::cout << "Error: " << tmp_tmLeftHandCart->getTaskError().transpose() << "\n" << std::endl;
    }

// }
