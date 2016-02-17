#ifndef FLOATINGBASEESTIMATIONTESTS_H
#define FLOATINGBASEESTIMATIONTESTS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class FloatingBaseEstimationTests : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
    };


// }


#endif
