#ifndef NOMINALPOSE_H
#define NOMINALPOSE_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {


    class NominalPose : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            // Full posture task
            Eigen::VectorXd                                 q_init;
            Eigen::VectorXd                                 nominal_q;
            double                                          tInitial;
            double                                          tFinal;
            int                                             t_pich_index;

    };

// }


#endif
