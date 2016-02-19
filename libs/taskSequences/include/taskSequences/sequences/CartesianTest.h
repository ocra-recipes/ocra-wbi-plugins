#ifndef CARTESIANTEST_H
#define CARTESIANTEST_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {

    class CartesianTest : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
            // ocra::FullPostureTaskManager*            tmFull;
            // // Partial posture task
            // ocra::PartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // ocra::SegCartesianTaskManager*           tmLeftHandCart;

            Eigen::Vector3d desiredPos;
            int lHandIndex;
     };

// }


#endif
