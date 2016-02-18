#ifndef ORIENTATIONTEST_H
#define ORIENTATIONTEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class OrientationTest : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
            // wocra::wOcraFullPostureTaskManager*            tmFull;
            // // Partial posture task
            // wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // wocra::wOcraSegOrientationTaskManager*         tmLeftHandOrient;

            Eigen::Rotation3d startingRotd;
            Eigen::Rotation3d endingRotd;
            int lHandIndex;
    };

// }


#endif
