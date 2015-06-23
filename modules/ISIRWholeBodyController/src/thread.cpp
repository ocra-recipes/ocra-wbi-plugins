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
// #include <ISIRWholeBodyController/ocraWbiModel.h>
#include <ocraWbiPlugins/ocraWbiModel.h>

#include <modHelp/modHelp.h>
#include <iostream>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/BufferedPort.h>


//#include "wocra/Solvers/OneLevelSolver.h"
#include "wocra/Features/wOcraFeature.h"
#include "ocra/control/Feature.h"
#include "ocra/control/FullState.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/control/ControlEnum.h"


// #include "ISIRWholeBodyController/sequenceLibrary.h"
#include "taskSequences/sequenceLibrary.h"


using namespace ISIRWholeBodyController;
using namespace yarp::math;
using namespace yarpWbi;
// using namespace sequence;

#define ALL_JOINTS -1
#define DIM_DISP 3
#define DIM_TWIST 6
#define TORQUE_MIN -24
#define TORQUE_MAX 24
//#define HAND_FOOT_TASK 1
#define HAND_FOOT_TASK 0
#define TIME_MSEC_TO_SEC 0.001

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Constructor
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISIRWholeBodyControllerThread::ISIRWholeBodyControllerThread(string _name,
                                                             string _robotName,
                                                             int _period,
                                                             wholeBodyInterface *_wbi,
                                                             yarp::os::Property &_options,
                                                             string _startupTaskSetPath,
                                                             string _startupSequence,
                                                             bool _runInDebugMode)
    : RateThread(_period),
      name(_name),
      robotName(_robotName),
      robot(_wbi),
      options(_options),
      startupTaskSetPath(_startupTaskSetPath),
      startupSequence(_startupSequence),
      runInDebugMode(_runInDebugMode)
{
    bool isFreeBase = false;
    ocraModel = new ocraWbiModel(robotName, robot->getDoFs(), robot, isFreeBase);
    bool useReducedProblem = false;
    ctrl = new wocra::wOcraController("icubControl", *ocraModel, internalSolver, useReducedProblem);

    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());

    homePosture = Eigen::VectorXd::Zero(robot->getDoFs());
    debugPosture = Eigen::VectorXd::Zero(robot->getDoFs());
    getHomePosture(*ocraModel, homePosture);
    getNominalPosture(*ocraModel, debugPosture);



    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);

    fb_Hroot_Vector = yarp::sig::Vector(16, 0.0);
    fb_Troot_Vector = yarp::sig::Vector(6, 0.0);

    fb_torque.resize(robot->getDoFs());

    time_sim = 0;


    if (runInDebugMode)
    {
        if (ocraModel->hasFixedRoot()) {
            std::cout << "Loading fixed base minimal tasks..." << std::endl;
            baseSequence = LoadSequence("FixedBaseMinimalTasks");
            baseSequenceIsActive = true;
            std::cout << "\n\n\t------------------------------" << std::endl;
            std::cout << "\t  Running in DEBUG mode..." << std::endl;
            std::cout << "\t------------------------------\n" << std::endl;

            std::string portPrefix = "/WBC/debug";
            debugPort_in.open((portPrefix+":i").c_str());
            debugPort_out.open((portPrefix+":o").c_str());
            debugJointIndex = 0;
        }
        else{
            std::cout << "[ERR] Cannot run debug mode with floating base." << std::endl;
        }

    }
    else
    {
        //Create XML task set
        if (!startupTaskSetPath.empty()) {
            std::cout << "\nLoading tasks from XML file:\n" << startupTaskSetPath << "\n" << std::endl;
            //TODO: Implement XML parsing
            xmlSequenceIsActive = true;
        }
        else{
            std::cout << "No XML task set detected." << std::endl;
            xmlSequenceIsActive = false;
        }

        //Create cpp sequence
        if (!startupSequence.empty()) {
            std::cout << "\nLoading sequence:\n" << startupSequence << "\n" << std::endl;
            cppSequence = LoadSequence(startupSequence);
            cppSequenceIsActive = true;
        }
        else{
            std::cout << "No startup sequence detected." << std::endl;
            cppSequenceIsActive = false;
        }

        // Create base sequence
        if(!xmlSequenceIsActive && !cppSequenceIsActive){
            std::cout << "\nNo tasks or scenarios loaded on startup. Defaulting to standard initial tasks." << std::endl;
            if (!ocraModel->hasFixedRoot()) {
                std::cout << "Loading floating base minimal tasks..." << std::endl;
                baseSequence = LoadSequence("FloatingBaseMinimalTasks");
                baseSequenceIsActive = true;
            }
            else{
                std::cout << "Loading fixed base minimal tasks..." << std::endl;
                baseSequence = LoadSequence("FixedBaseMinimalTasks");
                baseSequenceIsActive = true;
            }
        }
        else{
            //TODO: make a check that looks at all of the tasks on the controller and determines whether or not there is a need for the baseSequence
            baseSequenceIsActive = false;
        }
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Init
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
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



    if (runInDebugMode)
    {
        robot->setControlMode(CTRL_MODE_POS, debugPosture.data(), ALL_JOINTS);
    }
    else
    {
        // Set all declared joints in module to TORQUE mode
        bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    }


    // Initialise the sequence

    if (baseSequenceIsActive) {baseSequence->init(*ctrl, *ocraModel);}

    // if (xmlSequenceIsActive) {xmlSequence->init(*ctrl, *ocraModel);}

    if (cppSequenceIsActive) {cppSequence->init(*ctrl, *ocraModel);}




	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Run
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ISIRWholeBodyControllerThread::run()
{
    /******************************************************************************************************
                                            Update dynamic model
    ******************************************************************************************************/

    // Move this to header so can resize once
    yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);
    bool res_torque = robot->getEstimates(ESTIMATE_JOINT_TORQUE, fb_torque.data(), ALL_JOINTS);


    // SET THE STATE (FREE FLYER POSITION/VELOCITY AND Q)
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


    /******************************************************************************************************
                                            Update task sequences
    ******************************************************************************************************/

    if (runInDebugMode)
    {
        yarp::os::Bottle *input = debugPort_in.read(false);
        if (input != NULL)
        {
            int tempDebugIndex = input->get(0).asInt();
            if (tempDebugIndex>=0 || tempDebugIndex<robot->getDoFs()) {
                std::cout << "\n-----\nNew joint received...\n" << std::endl;
                std::cout << "Returning joint: " << debugJointIndex << " to home position." << std::endl;
                robot->setControlMode(CTRL_MODE_POS, &debugPosture[debugJointIndex], debugJointIndex);
                debugJointIndex = tempDebugIndex;
                std::cout << "Now joint: " << debugJointIndex << " is now being tested in torque control.\n-----\n" << std::endl;
            }
            else{std::cout << "\n[ERR] (thread.run) The command you sent was not a valid joint index. Please use integers between 0 and "<< robot->getDoFs() << ".\n"<< std::endl;}

        }
    }
    else{

        if (baseSequenceIsActive) {baseSequence->update(time_sim, *ocraModel, NULL);}

        // if (xmlSequenceIsActive) {xmlSequence->update(time_sim, *ocraModel, NULL);}

        if (cppSequenceIsActive) {cppSequence->update(time_sim, *ocraModel, NULL);}
    }


    /******************************************************************************************************
                                Compute desired torque by calling the controller
    ******************************************************************************************************/
    Eigen::VectorXd eigenTorques = Eigen::VectorXd::Constant(ocraModel->nbInternalDofs(), 0.0);

	ctrl->computeOutput(eigenTorques);


    /******************************************************************************************************
                                        Threshold the computed torques
    ******************************************************************************************************/
    for(int i = 0; i < eigenTorques.size(); ++i)
    {
      if(eigenTorques(i) < TORQUE_MIN) eigenTorques(i) = TORQUE_MIN;
      else if(eigenTorques(i) > TORQUE_MAX) eigenTorques(i) = TORQUE_MAX;
    }

	  modHelp::eigenToYarpVector(eigenTorques, torques_cmd);



    /******************************************************************************************************
                                    Send the torques to the robot via WBI
    ******************************************************************************************************/

    if (runInDebugMode) {
        robot->setControlMode(CTRL_MODE_TORQUE, &torques_cmd[debugJointIndex], debugJointIndex);
    }
    else{
        robot->setControlReference(torques_cmd.data());
    }


    /******************************************************************************************************
                            Print out useful information at every "printPeriod" (ms)
    ******************************************************************************************************/
    printPeriod = 500;
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print
    if (printCountdown == 0)
    {
        if (runInDebugMode)
        {
            Bottle& output = debugPort_out.prepare();
            output.clear();

            output.addString("joint");
            output.addInt(debugJointIndex);
            output.addString("torque_desired");
            output.addDouble(torques_cmd[debugJointIndex]);
            output.addString("torque_measured");
            output.addDouble(fb_torque[debugJointIndex]);

            debugPort_out.write();
        }
        else if (!ocraModel->hasFixedRoot()){
            std::cout<< "\n---\nfb_Hroot:\n" << fb_Hroot_Vector(3) << " "<< fb_Hroot_Vector(7) << " "<< fb_Hroot_Vector(11) << std::endl;
            // std::cout << "root_link pos\n" << ocraModel->getSegmentPosition(ocraModel->getSegmentIndex("root_link")).getTranslation().transpose() << std::endl;
            std::cout<< "fb_Troot:\n" << fb_Troot_Vector(0) <<" "<< fb_Troot_Vector(1) <<" "<< fb_Troot_Vector(2) << "\n---\n";
            std::cout << "root_link vel\n" << ocraModel->getSegmentVelocity(ocraModel->getSegmentIndex("root_link")).getLinearVelocity().transpose() << std::endl;
        }

    }

    time_sim += TIME_MSEC_TO_SEC * getRate();
}

//*************************************************************************************************************************
void ISIRWholeBodyControllerThread::threadRelease()
{
    //bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, 0, ALL_JOINTS);
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, q_initial.data());

    //yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    //robot->setControlReference(torques_cmd.data());
}
