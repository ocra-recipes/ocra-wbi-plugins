#include <taskSequences/sequences/CartesianTest.h>
#include <ocraWbiPlugins/ocraWbiModel.h>


// namespace sequence{
    void CartesianTest::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
    {
        ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

        // Task Coeffs
        double Kp = 10.0;
        double Kd = 2.0 * sqrt(Kp);
        double wFullPosture = 0.001;
        double wPartialPosture = 0.01;
        double wLeftHandTask = 1.0;

        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);
        tmFull = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

        // Partial (torso) posture task
        Eigen::VectorXi torso_indices(3);
        Eigen::VectorXd torsoTaskPosDes(3);
        torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
        torsoTaskPosDes << M_PI / 18, 0, 0;
        tmPartialTorso = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, wPartialPosture, torsoTaskPosDes);

        lHandIndex = model.getSegmentIndex("l_hand");

        Eigen::Vector3d YZ_disp;
        YZ_disp << 0.0, 0.2, 0.2;


        Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);

        // start and end positions
        Eigen::Vector3d startingPos = startingDispd.getTranslation();
        desiredPos  = startingPos+ YZ_disp;


        // Left hand cartesian task
        tmLeftHandCart = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp, Kd, wLeftHandTask, desiredPos);

    }

    void CartesianTest::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
        // tmLeftHandCart->setPosition(desiredPos);
        std::cout << "\n---\nDesired position: " << desiredPos.transpose() << std::endl;
        std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
        std::cout << "Error: " << tmLeftHandCart->getTaskError().transpose() << "\n" << std::endl;
    }

// }
