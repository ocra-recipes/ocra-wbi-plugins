#ifndef FLOATINGBASEMINIMALTASKS_H
#define FLOATINGBASEMINIMALTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class FloatingBaseMinimalTasks: public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& c, ocra::Model& m);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            wocra::wOcraController*                        ctrl;
            ocra::Model*                             model;
            wocra::wOcraCoMTaskManager*                    tmCoM;

    };


// }


#endif
