#include <taskSequences/sequences/TrajectoryTrackingTest.h>
#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"
// TrajectoryTrackingTest
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void TrajectoryTrackingTest::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
    {
        ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

        // Task Coeffs
        double Kp = 10.0;
        double Kd = 2.0 * sqrt(Kp);

        double Kp_hand = 40.0;
        double Kd_hand = 8.0 ;//* sqrt(Kp_hand);
        double wFullPosture = 0.0001;
        double wPartialPosture = 0.1;
        double wLeftHandTask = 1.0;

        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);
        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

        // Partial (torso) posture task
        Eigen::VectorXi torso_indices(3);
        Eigen::VectorXd torsoTaskPosDes(3);
        torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
        torsoTaskPosDes << 0, -10.0*(M_PI / 180.0), 40.0*(M_PI / 180.0);
        // torsoTaskPosDes << 0.0, 0.0, 0.0;
        taskManagers["tmPartialTorso"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 6., 2.0 * sqrt(6.), wPartialPosture, torsoTaskPosDes);

        // Right hand cartesian task
        Eigen::Vector3d posRHandDesDelta(0.1, 0.08, 0.15);

        Eigen::Vector3d posRHandDes = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
        posRHandDes = posRHandDes + posRHandDesDelta;


        taskManagers["tmSegCartHandRight"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, 1.0, posRHandDes);

        /**
        * Left hand task. Pick one of these booleans to test the different constructors.
        */
        //*************** Type of Trajectory ******************//
        bool isLinInterp = false;
        bool isMinJerk = true;
        //*****************************************************//

        //***************** Type of Reference ******************//
        isDisplacementd         = false;
        isRotation3d            = false;
        isCartesion             = false;
        isCartesionWaypoints    = true;
        //*****************************************************//
        int boolSum = isCartesion + isCartesionWaypoints + isDisplacementd + isRotation3d;
        if (boolSum>1){std::cout << "\nYou picked too many reference types." << std::endl;}
        else if (boolSum<1){std::cout << "\nYou picked too few reference types." << std::endl;}

        lHandIndex = model.getSegmentIndex("l_hand");

        Eigen::Vector3d YZ_disp;
        YZ_disp << -0.15, 0.1, 0.2;

        // start and end displacements & rotations
        Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);
        Eigen::Rotation3d startingRotd      = startingDispd.getRotation();

        endingRotd  = Eigen::Rotation3d(0.105135,   0.0828095,   0.253438,    -0.958049);
        endingDispd = Eigen::Displacementd(startingDispd.getTranslation() + YZ_disp, endingRotd);

        // start and end positions
        Eigen::Vector3d startingPos = startingDispd.getTranslation();
        desiredPos  = startingPos + YZ_disp;

        // multiple position waypoints
        Eigen::MatrixXd waypoints(3,5);
        Eigen::MatrixXd squareDisplacement(3,5);
        waypoints << startingPos, startingPos, startingPos, startingPos, startingPos;
        squareDisplacement << 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.2, 0.2, 0.0, 0.0,
                              0.0, 0.0, 0.2, 0.2, 0.0;
        waypoints += squareDisplacement;

        if (isLinInterp)
        {
            /**
            * Linear interpolation trajectory constructor tests:
            */
            if      (isDisplacementd)       {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingDispd, endingDispd);}
            else if (isRotation3d)          {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingRotd, endingRotd);}
            else if (isCartesion)           {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingPos, desiredPos);}
            else if (isCartesionWaypoints)  {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(waypoints);}
            else                            {std::cout << "\nGotta pick a reference type motherfucker!" << std::endl;}
        }
        else if (isMinJerk)
        {
            /**
            * Minimum jerk trajectory constructor tests:
            */
            if      (isDisplacementd)       {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingDispd, endingDispd);}
            else if (isRotation3d)          {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingRotd, endingRotd);}
            else if (isCartesion)           {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingPos, desiredPos);}
            else if (isCartesionWaypoints)  {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(waypoints);}
            else                            {std::cout << "\nGotta pick a reference type motherfucker!" << std::endl;}
        }
        else{std::cout << "\nGotta pick a trajectory type motherfucker!" << std::endl;}


        leftHandTrajectory->generateTrajectory(3.0); // set a 4 second duration

        if      (isDisplacementd)      {taskManagers["tmLeftHandPose"]      = new wocra::wOcraSegPoseTaskManager(ctrl, model, "leftHandPoseTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingDispd);}
        else if (isRotation3d)         {taskManagers["tmLeftHandOrient"]    = new wocra::wOcraSegOrientationTaskManager(ctrl, model, "leftHandOrientationTask", "l_hand", Kp_hand, Kd_hand, wLeftHandTask, startingRotd);}
        else if (isCartesion)          {taskManagers["tmLeftHandCart"]      = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingPos);}
        else if (isCartesionWaypoints) {taskManagers["tmLeftHandCart"]      = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingPos);}


    // tmSegCartHandRight = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, posRHandDes);
    }



    void TrajectoryTrackingTest::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
        if (isDisplacementd)
        {
            wocra::wOcraSegPoseTaskManager*   tmp_tmLeftHandPose = dynamic_cast<wocra::wOcraSegPoseTaskManager*>(taskManagers["tmLeftHandPose"]);

            Eigen::Displacementd desiredPose;
            Eigen::Twistd desiredVelocity;
            Eigen::Twistd desiredAcceleration;
            leftHandTrajectory->getDesiredValues(time, desiredPose, desiredVelocity, desiredAcceleration);

            tmp_tmLeftHandPose->setState(desiredPose, desiredVelocity, desiredAcceleration);

            // tmp_tmLeftHandPose->setState(desiredPose, desiredVelocity, Eigen::Twistd::Zero());

            std::cout << "\nFinal desired pose: " << endingDispd << std::endl;
            std::cout << "Desired pose: " << desiredPose << std::endl;
            std::cout << "Desired vel: " << desiredVelocity.transpose() << std::endl;
            std::cout << "Desired acc: " << desiredAcceleration.transpose() << std::endl;
            std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex) << std::endl;
            std::cout << "Error: " << tmp_tmLeftHandPose->getTaskError().transpose() << "   norm: " << tmp_tmLeftHandPose->getTaskErrorNorm() << std::endl;

        }
        else if (isRotation3d)
        {

            wocra::wOcraSegOrientationTaskManager*   tmp_tmLeftHandOrient = dynamic_cast<wocra::wOcraSegOrientationTaskManager*>(taskManagers["tmLeftHandOrient"]);


            Eigen::Rotation3d desiredOrientation;
            leftHandTrajectory->getDesiredValues(time, desiredOrientation);
            tmp_tmLeftHandOrient->setOrientation(desiredOrientation);

            std::cout << "\nFinal desired orientation: " << endingRotd << std::endl;
            std::cout << "Desired orientation: " << desiredOrientation << std::endl;
            std::cout << "Current orientation: " << state.getSegmentPosition(lHandIndex).getRotation() << std::endl;
            std::cout << "Error: " << tmp_tmLeftHandOrient->getTaskError().transpose() << "   norm: " << tmp_tmLeftHandOrient->getTaskErrorNorm() << std::endl;
        }
        else if (isCartesion || isCartesionWaypoints)
        {
            wocra::wOcraSegCartesianTaskManager*   tmp_tmLeftHandCart = dynamic_cast<wocra::wOcraSegCartesianTaskManager*>(taskManagers["tmLeftHandCart"]);


            Eigen::MatrixXd desiredPosVelAcc = leftHandTrajectory->getDesiredValues(time);
            // Eigen::MatrixXd H_adj = state.getSegmentPosition(lHandIndex).getRotation().inverse().adjoint();
            // H_adj*
            tmp_tmLeftHandCart->setState(desiredPosVelAcc.col(0));//,  desiredPosVelAcc.col(1), desiredPosVelAcc.col(2));

            // if(isCartesion){std::cout << "\nFinal desired position: " << desiredPos.transpose() << std::endl;}
            // std::cout << "\nDesired position: " << desiredPosVelAcc.col(0).transpose() << std::endl;
            // std::cout << "Current position: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
            // std::cout << "Error: " << tmp_tmLeftHandCart->getTaskError().transpose() << "   norm: " << tmp_tmLeftHandCart->getTaskErrorNorm() << std::endl;
        }



    }
// }
