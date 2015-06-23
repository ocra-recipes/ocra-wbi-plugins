#ifndef FLOATINGBASEMINIMALTASKS_H
#define FLOATINGBASEMINIMALTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class FloatingBaseMinimalTasks: public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);

    };


// }


#endif
