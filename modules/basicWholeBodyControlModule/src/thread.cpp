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
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


using namespace basicWholeBodyControlNamespace;
using namespace yarp::math;
using namespace wbiIcub;


//*************************************************************************************************************************
basicWholeBodyControlThread::basicWholeBodyControlThread(string _name,
                                                             string _robotName,
                                                             int _period,
                                                             wholeBodyInterface *_wbi,
                                                             yarp::os::Property &_options
                                                            )
    : RateThread(_period), name(_name), robotName(_robotName), robot(_wbi), options(_options)
{
    printCountdown = 0;
}

//*************************************************************************************************************************
bool basicWholeBodyControlThread::threadInit()
{
    qRad.resize(robot->getDoFs());
    printPeriod = options.check("printPeriod",Value(1000.0),"Print a debug message every printPeriod milliseconds.").asDouble();

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, -1);
    return true;
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::run()
{
    // Move this to header so can resize once
    yarp::sig::Vector torques;
    yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 10.0);
    torques.resize(robot->getDoFs());
    bool res = robot->getEstimates(ESTIMATE_JOINT_POS, qRad.data(), -1.0);
    bool res2 = robot->getEstimates(ESTIMATE_JOINT_TORQUE, torques.data(), -1.0);

    // prepare the matrices for WBC

    // compute desired torque from WBC

    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

    if(printCountdown==0) {
        std::cout << "The robot encoders values are: " << std::endl;
        std::cout << qRad.toString() << std::endl;
        std::cout << "The joint torquess are" << std::endl;
        std::cout << torques.toString() << std::endl;
    }
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::threadRelease()
{
}

