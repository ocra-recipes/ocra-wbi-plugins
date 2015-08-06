#ifndef FIXEDBASEMINIMALTASKS_H
#define FIXEDBASEMINIMALTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"
#include <math.h>
#include <vector>

#define NB_POSTURES 5
#define PERIOD 10.0

// namespace sequence {

    class FixedBaseMinimalTasks : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            wocra::wOcraFullPostureTaskManager *posture;
            // Full posture task
            int mode; // up down
            std::vector<Eigen::VectorXd> q;
            // PD parameters values
            double p;
            double t_d;
            double w;
            int count;
    };

// }


#endif
