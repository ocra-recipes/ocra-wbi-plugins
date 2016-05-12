#ifndef TRAJECTORYTRACKINGTEST_H
#define TRAJECTORYTRACKINGTEST_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

#include "ocra/control/Trajectory/Trajectory.h"

// namespace sequence {

    class TrajectoryTrackingTest : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // // Full posture task
            // ocra::FullPostureTaskManager*            tmFull;
            // // Partial posture task
            // ocra::PartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // ocra::SegCartesianTaskManager*           tmLeftHandCart;
            // ocra::SegPoseTaskManager*                tmLeftHandPose;
            // ocra::SegOrientationTaskManager*         tmLeftHandOrient;
            //
            // ocra::SegCartesianTaskManager*           tmSegCartHandRight;

            // trajectory
            ocra::Trajectory*                        leftHandTrajectory;

            Eigen::Displacementd endingDispd;
            Eigen::Rotation3d endingRotd;
            Eigen::Vector3d desiredPos;
            int lHandIndex;
            bool isDisplacementd;
            bool isRotation3d;
            bool isCartesion;
            bool isCartesionWaypoints;
    };

// }


#endif
