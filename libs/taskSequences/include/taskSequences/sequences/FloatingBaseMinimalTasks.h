#ifndef FLOATINGBASEMINIMALTASKS_H
#define FLOATINGBASEMINIMALTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class FloatingBaseMinimalTasks: public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& c, wocra::wOcraModel& m);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            wocra::wOcraController*                        ctrl;
            wocra::wOcraModel*                             model;
            wocra::wOcraCoMTaskManager*                    tmCoM;

    };


// }


#endif
