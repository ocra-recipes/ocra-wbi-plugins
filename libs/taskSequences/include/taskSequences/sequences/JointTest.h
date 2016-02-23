#ifndef JOINTTEST_H
#define JOINTTEST_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {

    class JointTest : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);

            Eigen::VectorXd q_init;
            Eigen::VectorXd q_des;
            Eigen::VectorXd jointMin;
            Eigen::VectorXd jointMax;
            int nDoF;
            int jIndex;
            double taskErr;
            bool goToMin;
            bool goToMax;


            std::string jointNames [25];
            int counter;


        private:
            // Full posture task

    };

// }


#endif
