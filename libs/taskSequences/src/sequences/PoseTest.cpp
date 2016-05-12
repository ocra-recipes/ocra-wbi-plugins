#include <taskSequences/sequences/PoseTest.h>


// namespace sequence{
    void PoseTest::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        // ocraWbiModel& model = dynamic_cast<ocraWbiModel&>(model);

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
        std::cout << startingDispd << std::endl;
        // start and end positions
        Eigen::Vector3d desiredPos = startingDispd.getTranslation() + YZ_disp;
        // Eigen::Rotation3d desiredOrientation = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        Eigen::Rotation3d desiredOrientation = startingDispd.getRotation();

        endingDispd    = Eigen::Displacementd(desiredPos, desiredOrientation);//, startingDispd.getRotation());//.inverse());

        // endingDispd    = startingDispd;
        std::cout << endingDispd << std::endl;
        // Left hand cartesian task
        taskManagers["tmLeftHandPose"]      = std::make_shared<ocra::SegPoseTaskManager>(ctrl, model, "leftHandPoseTask", "l_hand", ocra::XYZ, Kp, Kd, wLeftHandTask, endingDispd);
    }

    void PoseTest::doUpdate(double time, ocra::Model& state, void** args)
    {
        ocra::SegPoseTaskManager*   tmp_tmLeftHandPose = dynamic_cast<ocra::SegPoseTaskManager*>(taskManagers["tmLeftHandPose"].get());
        // tmLeftHandCart->setPosition(desiredPos);
        std::cout << "\n---\nDesired pose: " << endingDispd << std::endl;
        std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex)<< std::endl;
        std::cout << "Error: " << tmp_tmLeftHandPose->getTaskError().transpose() << "\n" << std::endl;
        // tmLeftHandPose->printTaskErrors();
}
// }
