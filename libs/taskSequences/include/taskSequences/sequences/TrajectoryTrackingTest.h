#ifndef TRAJECTORYTRACKINGTEST_H
#define TRAJECTORYTRACKINGTEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

#include "wocra/Trajectory/wOcraTrajectory.h"

// namespace sequence {

    class TrajectoryTrackingTest : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            // // Full posture task
            // wocra::wOcraFullPostureTaskManager*            tmFull;
            // // Partial posture task
            // wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // wocra::wOcraSegCartesianTaskManager*           tmLeftHandCart;
            // wocra::wOcraSegPoseTaskManager*                tmLeftHandPose;
            // wocra::wOcraSegOrientationTaskManager*         tmLeftHandOrient;
            //
            // wocra::wOcraSegCartesianTaskManager*           tmSegCartHandRight;

            // trajectory
            wocra::wOcraTrajectory*                        leftHandTrajectory;

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
