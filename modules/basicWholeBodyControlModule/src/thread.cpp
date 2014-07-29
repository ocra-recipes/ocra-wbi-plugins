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
#include <modHelp/modHelp.h>
#include <iostream>

#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


//#include "orcisir/Solvers/OneLevelSolver.h"
#include "orcisir/Features/ISIRFeature.h"
#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"


using namespace basicWholeBodyControlNamespace;
using namespace yarp::math;
using namespace wbiIcub;

#define ALL_JOINTS -1
#define DIM_DISP 3
#define DIM_TWIST 6

// Task Sets
//#include "taskSet1.h"
#include "taskSetTests.h"

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
    bool useReducedProblem = false;
    ctrl = new orcisir::ISIRController("icubControl", *orcModel, internalSolver, useReducedProblem);
    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);
    fb_torque.resize(robot->getDoFs());

    printCountdown = 0;
}

//*************************************************************************************************************************
bool basicWholeBodyControlThread::threadInit()
{
    printPeriod = options.check("printPeriod",Value(1000.0),"Print a debug message every printPeriod milliseconds.").asDouble();

    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);
    if (!orcModel->hasFixedRoot())
        orcModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    else
        orcModel->setState(fb_qRad, fb_qdRad);

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    
    
    tau_max = 12.0;
    tau_min = -12.0;
    
    
    //================ SET UP TASK ===================//
    //taskManager = TaskSet1::getTask(*orcModel, *ctrl);
    
	//taskManager = TaskSet_initialPosHold::getTask(*orcModel, *ctrl);
	//taskManager = TaskSet_initialPosZero::getTask(*orcModel, *ctrl);
//	taskManager = TaskSet_initialPosHold_leftHandPos::getTask(*orcModel, *ctrl);
    taskManager = TaskSet_initialPosHold_CoMPos_BothHandPos::getTask(*orcModel, *ctrl);
	
	
	return true;
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::run()
{
    std::cout << "Running Control Loop" << std::endl;

    // Move this to header so can resize once
    yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);
    bool res_torque = robot->getEstimates(ESTIMATE_JOINT_TORQUE, fb_torque.data(), ALL_JOINTS);

    // SET THE STATE (FREE FLYER POSITION/VELOCITY AND Q)
    if (!orcModel->hasFixedRoot())
        orcModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    else    
        orcModel->setState(fb_qRad, fb_qdRad);

    // compute desired torque by calling the controller
    Eigen::VectorXd eigenTorques = Eigen::VectorXd::Constant(orcModel->nbInternalDofs(), 0.0);
	ctrl->computeOutput(eigenTorques);

//    std::cout << "task error:" << std::endl;
//    std::cout << ctrl->getTask("accTask").getError() << std::endl;
//    std::cout << "torque:" << std::endl;
//    std::cout << fb_torque.toString() << std::endl;
//    std::cout << eigenTorques.transpose() << std::endl;


    for(int i = 0; i < eigenTorques.size(); ++i)
    {
      if(eigenTorques(i) > tau_max) eigenTorques(i) = tau_max;
      else if(eigenTorques(i) < tau_min) eigenTorques(i) = tau_min;
    }

	modHelp::eigenToYarpVector(eigenTorques, torques_cmd);


    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

    if(printCountdown==0) {
        //std::cout << "The robot encoders values are: " << std::endl;
        //std::cout << fb_qRad.transpose() << std::endl;
        //std::cout << "The joint torquess are" << std::endl;
        //std::cout << fb_torque.toString() << std::endl;


        //std::cout << "Data in orcModel" << std::endl;
        //orcModel->printAllData();


    }
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::threadRelease()
{
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, 0, ALL_JOINTS);
    //yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    //robot->setControlReference(torques_cmd.data());
}
