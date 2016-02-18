#ifndef CARTESIANTEST_H
#define CARTESIANTEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class CartesianTest : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
            // wocra::wOcraFullPostureTaskManager*            tmFull;
            // // Partial posture task
            // wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
            // // Segment left hand task
            // wocra::wOcraSegCartesianTaskManager*           tmLeftHandCart;

            Eigen::Vector3d desiredPos;
            int lHandIndex;
     };

// }


#endif
