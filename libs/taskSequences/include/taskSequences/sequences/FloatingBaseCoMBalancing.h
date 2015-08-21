#ifndef FLOATINGBASECOMBALANCING_H
#define FLOATINGBASECOMBALANCING_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/Constraints/JointLimitConstraint.h"
#include "wocra/Constraints/TorqueLimitConstraint.h"
#include "../sequenceTools.h"

#include <fstream>

// namespace sequence {


    class FloatingBaseCoMBalancing: public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& c, wocra::wOcraModel& m);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            wocra::wOcraController*                        ctrl;
            wocra::wOcraModel*                             model;
            wocra::wOcraCoMTaskManager*                    tmCoM;

            //Limits
            wocra::JointLimitConstraint*                   jlConstraint;
            Eigen::VectorXd                                torqueSaturationLimit; /* actuatedDOFs */
            wocra::TorqueLimitConstraint*                  tauLimitConstraint;

            std::ofstream                                  datafile;
            std::vector<double>                            actual_com_y;
            std::vector<double>                            ref_com_y;
            bool                                           recorded;
            void sinusoidalTraj(double left, double right, double period, double t, double& posTraj, double& velTraj, double& accTraj);
            void saveCoMData();
            void setJointTorqueLimits();
            void setJointLimits(double hpos);

    };


// }


#endif
