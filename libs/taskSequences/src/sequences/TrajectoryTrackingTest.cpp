#include <taskSequences/sequences/TrajectoryTrackingTest.h>
#include "ocra/control/Trajectory/MinimumJerkTrajectory.h"
#include "ocra/control/Trajectory/LinearInterpolationTrajectory.h"
// TrajectoryTrackingTest


// namespace sequence{
    void TrajectoryTrackingTest::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        // ocraWbiModel& model = dynamic_cast<ocraWbiModel&>(model);

        // Task Coeffs
        double Kp = 5.0;
        double Kd = 2.0 * sqrt(Kp);

        double Kp_hand = 10.0;
        double Kd_hand = 2.0 *sqrt(Kp_hand);
        double wFullPosture = 0.0001;
        double wPartialPosture = 0.1;
        double wLeftHandTask = 1.0;

        // Full posture task
        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);
        taskManagers["fullPostureTask"] = std::make_shared<ocra::FullPostureTaskManager>(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

        // Partial (torso) posture task
        Eigen::VectorXi torso_indices(3);
        Eigen::VectorXd torsoTaskPosDes(3);
        torso_indices << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");
        torsoTaskPosDes << 0, -10.0*(M_PI / 180.0), 40.0*(M_PI / 180.0);
        // torsoTaskPosDes << 0.0, 0.0, 0.0;
        taskManagers["torsoPostureTask"] = std::make_shared<ocra::PartialPostureTaskManager>(ctrl, model, "torsoPostureTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, wPartialPosture, torsoTaskPosDes);

        // Right hand cartesian task
        Eigen::Vector3d posRHandDesDelta(0.1, 0.08, 0.15);

        Eigen::Vector3d posRHandDes = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
        posRHandDes = posRHandDes + posRHandDesDelta;


        taskManagers["rightHandCartesianTask"] = std::make_shared<ocra::SegCartesianTaskManager>(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, 1.0, posRHandDes);

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
                              0.0, 0.15, 0.15, 0.0, 0.0,
                              0.0, 0.0, 0.25, 0.25, 0.0;
        waypoints += squareDisplacement;

        if (isLinInterp)
        {
            /**
            * Linear interpolation trajectory constructor tests:
            */
            leftHandTrajectory = new ocra::LinearInterpolationTrajectory();

            if      (isDisplacementd)       {leftHandTrajectory->setWaypoints(startingDispd, endingDispd);}
            else if (isRotation3d)          {leftHandTrajectory->setWaypoints(startingRotd, endingRotd);}
            else if (isCartesion)           {leftHandTrajectory->setWaypoints(startingPos, desiredPos);}
            else if (isCartesionWaypoints)  {leftHandTrajectory->setWaypoints(waypoints);}
            else                            {std::cout << "\nGotta pick a reference type!" << std::endl;}
        }
        else if (isMinJerk)
        {
            /**
            * Minimum jerk trajectory constructor tests:
            */
            leftHandTrajectory = new ocra::MinimumJerkTrajectory();

            if      (isDisplacementd)       {leftHandTrajectory->setWaypoints(startingDispd, endingDispd);}
            else if (isRotation3d)          {leftHandTrajectory->setWaypoints(startingRotd, endingRotd);}
            else if (isCartesion)           {leftHandTrajectory->setWaypoints(startingPos, desiredPos);}
            else if (isCartesionWaypoints)  {leftHandTrajectory->setWaypoints(waypoints);}
            else                            {std::cout << "\nGotta pick a reference type motherfucker!" << std::endl;}
        }
        else{std::cout << "\nGotta pick a trajectory type!" << std::endl;}



        if      (isDisplacementd)      {taskManagers["leftHandPoseTask"]      = std::make_shared<ocra::SegPoseTaskManager>(ctrl, model, "leftHandPoseTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingDispd);}
        else if (isRotation3d)         {taskManagers["leftHandOrientationTask"] = std::make_shared<ocra::SegOrientationTaskManager>(ctrl, model, "leftHandOrientationTask", "l_hand", Kp_hand, Kd_hand, wLeftHandTask, startingRotd);}
        else if (isCartesion)          {taskManagers["leftHandCartesianTask"]      = std::make_shared<ocra::SegCartesianTaskManager>(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingPos);}
        else if (isCartesionWaypoints) {taskManagers["leftHandCartesianTask"]      = std::make_shared<ocra::SegCartesianTaskManager>(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingPos);}



    }



    void TrajectoryTrackingTest::doUpdate(double time, ocra::Model& state, void** args)
    {
        if (isDisplacementd)
        {
            ocra::SegPoseTaskManager*   tmp_tmLeftHandPose = dynamic_cast<ocra::SegPoseTaskManager*>(taskManagers["leftHandPoseTask"].get());

            Eigen::Displacementd desiredPose;
            Eigen::Twistd desiredVelocity;
            Eigen::Twistd desiredAcceleration;
            leftHandTrajectory->getDesiredValues(time, desiredPose, desiredVelocity, desiredAcceleration);

            tmp_tmLeftHandPose->setState(desiredPose, desiredVelocity, desiredAcceleration);


        }
        else if (isRotation3d)
        {

            ocra::SegOrientationTaskManager*   tmp_tmLeftHandOrient = dynamic_cast<ocra::SegOrientationTaskManager*>(taskManagers["leftHandOrientationTask"].get());

            Eigen::Rotation3d desiredOrientation;
            leftHandTrajectory->getDesiredValues(time, desiredOrientation);
            tmp_tmLeftHandOrient->setOrientation(desiredOrientation);

        }
        else if (isCartesion || isCartesionWaypoints)
        {
            ocra::SegCartesianTaskManager*   tmp_tmLeftHandCart = dynamic_cast<ocra::SegCartesianTaskManager*>(taskManagers["leftHandCartesianTask"].get());

            Eigen::MatrixXd desiredPosVelAcc = leftHandTrajectory->getDesiredValues(time);

            tmp_tmLeftHandCart->setState(desiredPosVelAcc.col(0),  desiredPosVelAcc.col(1), desiredPosVelAcc.col(2));

        }



    }
// }
