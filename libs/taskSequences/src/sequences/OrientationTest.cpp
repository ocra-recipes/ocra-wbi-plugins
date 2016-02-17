#include <taskSequences/sequences/OrientationTest.h>
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void OrientationTest::doInit(wocra::wOcraController& ctrl, ocra::Model& model)
    {
        ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

        // Task Coeffs
        double Kp = 40.0;
        double Kd = 2.0 * sqrt(Kp);
        double wFullPosture = 0.001;
        double wPartialPosture = 0.01;
        double wLeftHandTask = 1.0;

        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);
        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

        // Partial (torso) posture task
        Eigen::VectorXi torso_indices(3);
        Eigen::VectorXd torsoTaskPosDes(3);
        torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
        torsoTaskPosDes << M_PI / 18, 0, 0;
        taskManagers["tmPartialTorso"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, wPartialPosture, torsoTaskPosDes);

        lHandIndex = model.getSegmentIndex("r_hand");

        Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);

        // start and end rotations
        startingRotd      = startingDispd.getRotation();
        // endingRotd        = Eigen::Rotation3d(0.105135,   0.0828095,   0.253438,    -0.958049);// palm down //startingRotd.inverse();
        std::cout << model.getSegmentPosition(model.getSegmentIndex("root_link")) << std::endl;

        double sqrt2on2 = sqrt(2.0)/2.0;
        Eigen::Rotation3d yToNegZ = Eigen::Rotation3d(sqrt2on2, 0.0, -sqrt2on2, 0.0);
        endingRotd        = startingRotd.inverse();// * yToNegZ;


        // Left hand orientation task
        taskManagers["tmLeftHandOrient"]    = new wocra::wOcraSegOrientationTaskManager(ctrl, model, "leftHandOrientationTask", "r_hand", Kp, Kd, Eigen::Vector3d(0.0, 1.0, 0.0), endingRotd);
    }

    void OrientationTest::doUpdate(double time, ocra::Model& state, void** args)
    {
        wocra::wOcraSegOrientationTaskManager*   tmp_tmLeftHandOrient = dynamic_cast<wocra::wOcraSegOrientationTaskManager*>(taskManagers["tmLeftHandOrient"]);
        std::cout << "\n---\nStarting orientation: " << startingRotd << std::endl;
        std::cout << "Desired orientation: " << endingRotd << std::endl;
        std::cout << "Current orientation: " << state.getSegmentPosition(lHandIndex).getRotation()<< std::endl;
        std::cout << "Error: " << tmp_tmLeftHandOrient->getTaskError().transpose() << "\n" << std::endl;
        // tmLeftHandOrient->printTaskErrors();
    }
// }
