/*
* Copyright (C) 2013 ISIR
* Author: Darwin Lau, MingXing Liu, Ryan Lober
* email: lau@isir.upmc.fr
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

#include <ISIRWholeBodyController/thread.h>
#include <ISIRWholeBodyController/orcWbiModel.h>
#include <modHelp/modHelp.h>
#include <iostream>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


//#include "orcisir/Solvers/OneLevelSolver.h"
#include "orcisir/Features/ISIRFeature.h"
#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"


using namespace ISIRWholeBodyController;
using namespace yarp::math;
using namespace yarpWbi;

#define ALL_JOINTS -1
#define DIM_DISP 3
#define DIM_TWIST 6
#define TORQUE_MIN -24
#define TORQUE_MAX 24
//#define HAND_FOOT_TASK 1
#define HAND_FOOT_TASK 0
#define TIME_MSEC_TO_SEC 0.001

//*************************************************************************************************************************
ISIRWholeBodyControllerThread::ISIRWholeBodyControllerThread(string _name,
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

    time_sim = 0;
}

//*************************************************************************************************************************
bool ISIRWholeBodyControllerThread::threadInit()
{
//    printPeriod = options.check("printPeriod",Value(1000.0),"Print a debug message every printPeriod milliseconds.").asDouble();
    robot->getEstimates(ESTIMATE_JOINT_POS, q_initial.data(), ALL_JOINTS);

    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);

    if (!orcModel->hasFixedRoot())
        orcModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    else
        orcModel->setState(fb_qRad, fb_qdRad);

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    
    //================ SET UP TASK ===================//
    //sequence = new Sequence_NominalPose();
    //sequence = new Sequence_InitialPoseHold();
    //sequence = new Sequence_LeftHandReach();
    //sequence = new Sequence_LeftRightHandReach();

    // sequence = new Sequence_CartesianTest;
    // sequence = new Sequence_PoseTest;
    // sequence = new Sequence_OrientationTest;
    

    sequence = new Sequence_TrajectoryTrackingTest();
    

    sequence->init(*ctrl, *orcModel);
	
	return true;
}

//*************************************************************************************************************************
void ISIRWholeBodyControllerThread::run()
{
//    std::cout << "Running Control Loop" << std::endl;

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

    sequence->update(time_sim, *orcModel, NULL);
    
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
      if(eigenTorques(i) < TORQUE_MIN) eigenTorques(i) = TORQUE_MIN;
      else if(eigenTorques(i) > TORQUE_MAX) eigenTorques(i) = TORQUE_MAX;
    }
      //std::cout << "\n--\nTorso Pitch Torque = " << eigenTorques(orcModel->getDofIndex("torso_pitch")) << "\n--\n" << std::endl;

	  modHelp::eigenToYarpVector(eigenTorques, torques_cmd);

    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printPeriod = 10000;
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print
    if (printCountdown == 0)
    {
        std::cout << "ISIRWholeBodyController thread running..." << std::endl;
    }
/*
    printPeriod = 5000;
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

    if(printCountdown==0 && HAND_FOOT_TASK) {
        //std::cout << "The robot encoders values are: " << std::endl;
        //std::cout << fb_qRad.transpose() << std::endl;
        //std::cout << "The joint torquess are" << std::endl;
        //std::cout << fb_torque.toString() << std::endl;


        //std::cout << "Data in orcModel" << std::endl;
        //orcModel->printAllData();
        std::cout << "task target 1" <<ctrl->getTask("l_hand_task").isActiveAsObjective()<< std::endl;
        std::cout << "task target 2" <<ctrl->getTask("l_hand_task2").isActiveAsObjective()<< std::endl;
        //lhand, rfoot, rshank
        if (ctrl->getTask("l_hand_task").isActiveAsObjective())
            ctrl->getTask("l_hand_task").deactivate();
        else
            ctrl->getTask("l_hand_task").activateAsObjective();

        if (ctrl->getTask("l_hand_task2").isActiveAsObjective())
            ctrl->getTask("l_hand_task2").deactivate();
        else
            ctrl->getTask("l_hand_task2").activateAsObjective();

        if (ctrl->getTask("r_shank_task").isActiveAsObjective())
            ctrl->getTask("r_shank_task").deactivate();
        else
            ctrl->getTask("r_shank_task").activateAsObjective();

        if (ctrl->getTask("r_shank_task2").isActiveAsObjective())
            ctrl->getTask("r_shank_task2").deactivate();
        else
            ctrl->getTask("r_shank_task2").activateAsObjective();
        //rhand, lfoot, lshank
        if (ctrl->getTask("r_hand_task").isActiveAsObjective())
            ctrl->getTask("r_hand_task").deactivate();
        else
            ctrl->getTask("r_hand_task").activateAsObjective();

        if (ctrl->getTask("r_hand_task2").isActiveAsObjective())
            ctrl->getTask("r_hand_task2").deactivate();
        else
            ctrl->getTask("r_hand_task2").activateAsObjective();

        if (ctrl->getTask("l_shank_task").isActiveAsObjective())
            ctrl->getTask("l_shank_task").deactivate();
        else
            ctrl->getTask("l_shank_task").activateAsObjective();

        if (ctrl->getTask("l_shank_task2").isActiveAsObjective())
            ctrl->getTask("l_shank_task2").deactivate();
        else
            ctrl->getTask("l_shank_task2").activateAsObjective();
    }
*/
    time_sim += TIME_MSEC_TO_SEC * getRate();
}

//*************************************************************************************************************************
void ISIRWholeBodyControllerThread::threadRelease()
{
    //bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, 0, ALL_JOINTS);
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, q_initial.data(), ALL_JOINTS);

    //yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    //robot->setControlReference(torques_cmd.data());
}
