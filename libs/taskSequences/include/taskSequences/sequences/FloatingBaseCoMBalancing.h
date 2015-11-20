#ifndef FLOATINGBASECOMBALANCING_H
#define FLOATINGBASECOMBALANCING_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/Constraints/JointLimitConstraint.h"
#include "wocra/Constraints/TorqueLimitConstraint.h"
#include "../sequenceTools.h"

#include <fstream>

//apply external wrench
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>

// namespace sequence {


    class FloatingBaseCoMBalancing: public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& c, wocra::wOcraModel& m);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
        private:
            wocra::wOcraController*                        ctrl;
            wocra::wOcraModel*                             model;

            //CoM task
            wocra::wOcraCoMTaskManager*                    tmCoM;
            Eigen::Vector3d                                initialCoMPosition;
            void sinusoidalTraj(double left, double right, double period, double t, double& posTraj, double& velTraj, double& accTraj);

            //Limits
            wocra::JointLimitConstraint*                   jlConstraint;
            Eigen::VectorXd                                torqueSaturationLimit;
            wocra::TorqueLimitConstraint*                  tauLimitConstraint;
            void setJointTorqueLimits();
            void setJointLimits(double hpos);

            //Record results
            std::ofstream                                  datafile;
            std::vector<double>                            actual_com_x;
            std::vector<double>                            ref_com_x;
            std::vector<double>                            actual_com_y;
            std::vector<double>                            ref_com_y;
            std::vector<double>                            flf_x;
            std::vector<double>                            flf_y;
            std::vector<double>                            flf_z;
            std::vector<double>                            frf_x;
            std::vector<double>                            frf_y;
            std::vector<double>                            frf_z;
            std::vector<double>                            com_momentum_error_x;
            std::vector<double>                            com_momentum_error_y;
            std::vector<double>                            com_momentum_error_z;
            bool                                           recorded;
            void saveCoMData();

            yarp::os::Network   yarpNet;

            //apply external wrench
            void applyExternalWrench(std::string linkName, Eigen::Vector3d& force, Eigen::Vector3d& torque, double duration);
            bool                                           applyWrench;

            //get contact force
            Eigen::Vector3d& getContactForce(yarp::os::RpcClient* p, std::string linkName);
            yarp::os::RpcClient portLFConctact;
            yarp::os::RpcClient portRFConctact;
            Eigen::Vector3d     force;


    };


// }


#endif
