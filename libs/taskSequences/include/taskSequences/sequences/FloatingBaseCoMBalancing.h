#ifndef FLOATINGBASECOMBALANCING_H
#define FLOATINGBASECOMBALANCING_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/Constraints/JointLimitConstraint.h"
#include "ocra/control/TorqueLimitConstraint.h"
#include "../sequenceTools.h"

#include <fstream>

// namespace sequence {


    class FloatingBaseCoMBalancing: public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& c, ocra::Model& m);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            wocra::wOcraController*                        ctrl;
            ocra::Model*                             model;

            //CoM task
            wocra::wOcraCoMTaskManager*                    tmCoM;
            void sinusoidalTraj(double left, double right, double period, double t, double& posTraj, double& velTraj, double& accTraj);

            //Limits
            wocra::JointLimitConstraint*                   jlConstraint;
            Eigen::VectorXd                                torqueSaturationLimit;
            ocra::TorqueLimitConstraint*                  tauLimitConstraint;
            void setJointTorqueLimits();
            void setJointLimits(double hpos);

            //Record results
            std::ofstream                                  datafile;
            std::vector<double>                            actual_com_y;
            std::vector<double>                            ref_com_y;
            bool                                           recorded;
            void saveCoMData();





    };


// }


#endif
