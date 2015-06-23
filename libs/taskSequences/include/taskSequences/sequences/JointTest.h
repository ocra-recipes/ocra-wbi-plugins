#ifndef JOINTTEST_H
#define JOINTTEST_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"

// namespace sequence {

    class JointTest : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);

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

            // ocraWbiModel& wbiModel;

        private:
            // Full posture task
            wocra::wOcraFullPostureTaskManager*            tmFull;

    };

// }


#endif
