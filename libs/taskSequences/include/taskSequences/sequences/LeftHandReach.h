#ifndef LEFTHANDREACH_H
#define LEFTHANDREACH_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class LeftHandReach : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            // Full posture task
            wocra::wOcraFullPostureTaskManager*            tmFull;
            // Segment left hand task
            wocra::wOcraSegCartesianTaskManager*           tmSegCartHandLeft;
    };

// }


#endif
