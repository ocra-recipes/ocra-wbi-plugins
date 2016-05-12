#ifndef POSETEST_H
#define POSETEST_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {


    class PoseTest : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
            // ocra::FullPostureTaskManager*            tmFull;
            // // Partial posture task
            // ocra::PartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // ocra::SegPoseTaskManager*                tmLeftHandPose;

            Eigen::Displacementd endingDispd;
            int lHandIndex;
    };

// }


#endif
