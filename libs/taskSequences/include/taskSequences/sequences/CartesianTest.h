#ifndef CARTESIANTEST_H
#define CARTESIANTEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class CartesianTest : public wocra::wOcraTaskSequenceBase
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
            wocra::wOcraSegCartesianTaskManager*           tmLeftHandCart;

            Eigen::Vector3d desiredPos;
            int lHandIndex;
     };

// }


#endif
