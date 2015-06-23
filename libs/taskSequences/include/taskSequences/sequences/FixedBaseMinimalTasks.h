#ifndef FIXEDBASEMINIMALTASKS_H
#define FIXEDBASEMINIMALTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class FixedBaseMinimalTasks : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            // Full posture task
            wocra::wOcraFullPostureTaskManager*            tmFull;
    };

// }


#endif
