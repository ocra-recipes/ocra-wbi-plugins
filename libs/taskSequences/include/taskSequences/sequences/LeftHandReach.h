#ifndef LEFTHANDREACH_H
#define LEFTHANDREACH_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class LeftHandReach : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
        
    };

// }


#endif
