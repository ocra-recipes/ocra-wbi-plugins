#ifndef LEFTRIGHTHANDREACH_H
#define LEFTRIGHTHANDREACH_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class LeftRightHandReach : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            // // Full posture task
            // wocra::wOcraFullPostureTaskManager*            tmFull;
            // // Partial posture task for torso
            // wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
            // // CoM task for torso
            // wocra::wOcraCoMTaskManager*                    tmCoM;
            // // Segment left hand task
            // wocra::wOcraSegCartesianTaskManager*           tmSegCartHandLeft;
            // // Segment right hand task
            // wocra::wOcraSegCartesianTaskManager*           tmSegCartHandRight;
    };

// }


#endif
