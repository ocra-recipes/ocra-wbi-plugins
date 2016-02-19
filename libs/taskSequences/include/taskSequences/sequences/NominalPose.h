#ifndef NOMINALPOSE_H
#define NOMINALPOSE_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {


    class NominalPose : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
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
