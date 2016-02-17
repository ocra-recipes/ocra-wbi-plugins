#ifndef POSETEST_H
#define POSETEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class PoseTest : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
            // wocra::wOcraFullPostureTaskManager*            tmFull;
            // // Partial posture task
            // wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // wocra::wOcraSegPoseTaskManager*                tmLeftHandPose;

            Eigen::Displacementd endingDispd;
            int lHandIndex;
    };

// }


#endif
