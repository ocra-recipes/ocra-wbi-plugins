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
#include <ISIRWholeBodyController/ocraWbiModel.h>
#include <modHelp/modHelp.h>
#include <iostream>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


//#include "wocra/Solvers/OneLevelSolver.h"
#include "wocra/Features/wOcraFeature.h"
#include "ocra/control/Feature.h"
#include "ocra/control/FullState.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/control/ControlEnum.h"


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
    ocraModel = new ocraWbiModel(robotName, robot->getDoFs(), robot, isFreeBase);
    bool useReducedProblem = false;
    ctrl = new wocra::wOcraController("icubControl", *ocraModel, internalSolver, useReducedProblem);

    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());
    
    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);

    fb_Hroot_Vector = yarp::sig::Vector(16, 0.0);
    fb_Troot_Vector = yarp::sig::Vector(6, 0.0);
    
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

    if (!ocraModel->hasFixedRoot()){
        // Get root position as a 12x1 vector and get root vel as a 6x1 vector
        bool res_fb_Hroot_Vector = robot->getEstimates(ESTIMATE_BASE_POS, fb_Hroot_Vector.data());
        bool res_fb_Troot = robot->getEstimates(ESTIMATE_BASE_VEL, fb_Troot_Vector.data());
        // Convert to a wbi::Frame and a "fake" Twistd
        wbi::frameFromSerialization(fb_Hroot_Vector.data(), fb_Hroot);
        fb_Troot = Eigen::Twistd(fb_Troot_Vector[0], fb_Troot_Vector[1], fb_Troot_Vector[2], fb_Troot_Vector[3], fb_Troot_Vector[4], fb_Troot_Vector[5]);
        
        ocraModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    }
    else
        ocraModel->setState(fb_qRad, fb_qdRad);

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    
    //================ SET UP TASK ===================//
    // sequence = new Sequence_NominalPose();
    // sequence = new Sequence_InitialPoseHold();
    // sequence = new Sequence_LeftHandReach();
    // sequence = new Sequence_LeftRightHandReach();

    // sequence = new Sequence_CartesianTest;
    // sequence = new Sequence_PoseTest;
    // sequence = new Sequence_OrientationTest;
    
    // sequence = new Sequence_TrajectoryTrackingTest();
    // sequence = new Sequence_JointTest();
    // sequence = new ScenarioICub_01_Standing();
    sequence = new ScenarioICub_02_VariableWeightHandTasks();
    
    
    
    // int lSoleIndex = ocraModel->getSegmentIndex("l_sole");
    // int rSoleIndex = ocraModel->getSegmentIndex("r_sole");

    // Eigen::Displacementd lSoleDisp = ocraModel->getSegmentPosition(lSoleIndex);
    // Eigen::Displacementd rSoleDisp = ocraModel->getSegmentPosition(rSoleIndex);
    // std::cout << "\n\n\nl_sole, index: " << lSoleIndex << " is at (x,y,z): " << lSoleDisp.getTranslation().transpose()  <<std::endl;

    // std::cout << "\nr_sole, index: " << rSoleIndex << " is at (x,y,z): " << rSoleDisp.getTranslation().transpose() <<std::endl;
    


    sequence->init(*ctrl, *ocraModel);
    // sequence_01->init(*ctrl, *ocraModel);
	
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


    // bool res_fb_Hroot_Vector = robot->getEstimates(ESTIMATE_BASE_POS, fb_Hroot_Vector.data());
    // bool res_fb_Troot = robot->getEstimates(ESTIMATE_BASE_VEL, fb_Troot_Vector.data());
    // std::cout<< "\n---\nfb_Hroot:\n" << fb_Hroot_Vector(0) << fb_Hroot_Vector(1) << fb_Hroot_Vector(2) << std::endl;
    // std::cout<< "fb_Troot:\n" << fb_Troot_Vector(0) << fb_Troot_Vector(1) << fb_Troot_Vector(2) << "\n---\n";



    // SET THE STATE (FREE FLYER POSITION/VELOCITY AND Q)
    if (!ocraModel->hasFixedRoot()){
        // Get root position as a 12x1 vector and get root vel as a 6x1 vector
        bool res_fb_Hroot_Vector = robot->getEstimates(ESTIMATE_BASE_POS, fb_Hroot_Vector.data());
        bool res_fb_Troot = robot->getEstimates(ESTIMATE_BASE_VEL, fb_Troot_Vector.data());
        // Convert to a wbi::Frame and a "fake" Twistd
        wbi::frameFromSerialization(fb_Hroot_Vector.data(), fb_Hroot);

        fb_Troot = Eigen::Twistd(fb_Troot_Vector[0], fb_Troot_Vector[1], fb_Troot_Vector[2], fb_Troot_Vector[3], fb_Troot_Vector[4], fb_Troot_Vector[5]);
        

        // std::cout << "H_root_wbi as a vector\n" << fb_Hroot_Vector.toString() <<"\n\n"<< std::endl;
        // std::cout << "H_root_wbi BEFORE input to Set State\n" << fb_Hroot.toString() <<"\n\n"<< std::endl;

        ocraModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    }
    else    
        ocraModel->setState(fb_qRad, fb_qdRad);

    sequence->update(time_sim, *ocraModel, NULL);
    // sequence_01->update(time_sim, *ocraModel, NULL);
    
    // compute desired torque by calling the controller
    Eigen::VectorXd eigenTorques = Eigen::VectorXd::Constant(ocraModel->nbInternalDofs(), 0.0);

	ctrl->computeOutput(eigenTorques);
    // std::cout << "torques: "<<eigenTorques.transpose() << std::endl;

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
      //std::cout << "\n--\nTorso Pitch Torque = " << eigenTorques(ocraModel->getDofIndex("torso_pitch")) << "\n--\n" << std::endl;

	  modHelp::eigenToYarpVector(eigenTorques, torques_cmd);

    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printPeriod = 1000;
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print
    if (printCountdown == 0)
    {
        if (!ocraModel->hasFixedRoot()){
            std::cout<< "\n---\nfb_Hroot:\n" << fb_Hroot_Vector(3) << " "<< fb_Hroot_Vector(7) << " "<< fb_Hroot_Vector(11) << std::endl;
            std::cout<< "fb_Troot:\n" << fb_Troot_Vector(0) <<" "<< fb_Troot_Vector(1) <<" "<< fb_Troot_Vector(2) << "\n---\n";
        }
        // std::cout << "l_ankle_pitch: " << fb_qRad(17) << "   r_ankle_pitch: " << fb_qRad(23) << std::endl;
        //std::cout << "ISIRWholeBodyController thread running..." << std::endl;
    }
/*
    printPeriod = 5000;
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

    if(printCountdown==0 && HAND_FOOT_TASK) {
        //std::cout << "The robot encoders values are: " << std::endl;
        //std::cout << fb_qRad.transpose() << std::endl;
        //std::cout << "The joint torquess are" << std::endl;
        //std::cout << fb_torque.toString() << std::endl;


        //std::cout << "Data in ocraModel" << std::endl;
        //ocraModel->printAllData();
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
