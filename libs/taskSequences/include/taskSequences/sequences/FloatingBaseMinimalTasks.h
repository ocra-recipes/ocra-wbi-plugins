#ifndef FLOATINGBASEMINIMALTASKS_H
#define FLOATINGBASEMINIMALTASKS_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {


    class FloatingBaseMinimalTasks: public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& c, ocra::Model& m);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            ocra::Model*                             model;
            ocra::CoMTaskManager*                    tmCoM;

    };


// }


#endif
