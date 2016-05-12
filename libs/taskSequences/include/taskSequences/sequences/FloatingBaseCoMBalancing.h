#ifndef FLOATINGBASECOMBALANCING_H
#define FLOATINGBASECOMBALANCING_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "ocra/control/TorqueLimitConstraint.h"
#include "ocra/control/JointLimitConstraint.h"
#include "../sequenceTools.h"

#include "wocra/WocraController.h"


#include <fstream>

// namespace sequence {


    class FloatingBaseCoMBalancing: public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& c, ocra::Model& m);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            wocra::WocraController*                        ctrl;
            ocra::Model*                             model;

            //CoM task
            ocra::CoMTaskManager*                    tmCoM;
            void sinusoidalTraj(double left, double right, double period, double t, double& posTraj, double& velTraj, double& accTraj);

            //Limits
            ocra::JointLimitConstraint*                   jlConstraint;
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
