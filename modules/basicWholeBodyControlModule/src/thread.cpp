/*
* Copyright (C) 2013 CoDyCo
* Author: Andrea Del Prete
* email: andrea.delprete@iit.it
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include "thread.h"
#include "orcWbiModel.h"

#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


using namespace basicWholeBodyControlNamespace;
using namespace yarp::math;
using namespace wbiIcub;

#define ALL_JOINTS -1
#define DIM_DISP 3
#define DIM_TWIST 6

//*************************************************************************************************************************
basicWholeBodyControlThread::basicWholeBodyControlThread(string _name,
                                                             string _robotName,
                                                             int _period,
                                                             wholeBodyInterface *_wbi,
                                                             yarp::os::Property &_options
                                                            )
    : RateThread(_period), name(_name), robotName(_robotName), robot(_wbi), options(_options)
{
    bool isFreeBase = false;
    orcModel = new orcWbiModel(robotName, robot->getDoFs(), robot, isFreeBase);
    printCountdown = 0;
}

//*************************************************************************************************************************
bool basicWholeBodyControlThread::threadInit()
{
    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_Hroot = Eigen::Vector3d::Zero();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);
    fb_torque.resize(robot->getDoFs());
    printPeriod = options.check("printPeriod",Value(1000.0),"Print a debug message every printPeriod milliseconds.").asDouble();

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::run()
{
    // Move this to header so can resize once
    yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);
    bool res_torque = robot->getEstimates(ESTIMATE_JOINT_TORQUE, fb_torque.data(), ALL_JOINTS);

    // TEMPORARY : set H_root and T_root as zero


    // SET THE FREE FLYER POSITION/VELOCITY AND Q HERE
    std::cout << "THREAD IS SETTING STATE\n";
    std::cout << "Q" << std::endl;
    std::cout << fb_qRad.transpose() << std::endl;
    std::cout << "QD" << std::endl;
    std::cout << fb_qdRad.transpose() << std::endl;
    orcModel->setState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    std::cout << "THREAD HAS SET STATE\n";

        std::cout << "Data in orcModel" << std::endl;
    orcModel->printAllData();

    // compute desired torque by calling the controller

    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

/*
    if(printCountdown==0) {
        std::cout << "The robot encoders values are: " << std::endl;
        std::cout << fb_qRad << std::endl;
        std::cout << "The joint torquess are" << std::endl;
        std::cout << fb_torque.toString() << std::endl;
        

        std::cout << "Data in orcModel" << std::endl;
        orcModel->printAllData();
        orcModel->printAllCoMData();

    }
*/
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::threadRelease()
{
}
