#ifndef LEFTRIGHTHANDREACH_H
#define LEFTRIGHTHANDREACH_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {

    class LeftRightHandReach : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // // Full posture task
            // ocra::FullPostureTaskManager*            tmFull;
            // // Partial posture task for torso
            // ocra::PartialPostureTaskManager*         tmPartialTorso;
            // // CoM task for torso
            // ocra::CoMTaskManager*                    tmCoM;
            // // Segment left hand task
            // ocra::SegCartesianTaskManager*           tmSegCartHandLeft;
            // // Segment right hand task
            // ocra::SegCartesianTaskManager*           tmSegCartHandRight;
    };

// }


#endif
