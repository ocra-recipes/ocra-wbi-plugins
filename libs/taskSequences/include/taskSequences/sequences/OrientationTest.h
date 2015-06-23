#ifndef ORIENTATIONTEST_H
#define ORIENTATIONTEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class OrientationTest : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            // Full posture task
            wocra::wOcraFullPostureTaskManager*            tmFull;
            // Partial posture task
            wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
            // Segment left hand task
            wocra::wOcraSegOrientationTaskManager*         tmLeftHandOrient;

            Eigen::Rotation3d startingRotd;
            Eigen::Rotation3d endingRotd;
            int lHandIndex;
    };

// }


#endif
