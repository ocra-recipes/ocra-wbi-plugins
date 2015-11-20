/*
* Copyright (C) 2013 ISIR
* Author: Darwin Lau, MingXing Liu, Ryan Lober
* email: lau@isir.upmc.fr, liu@isir.upmc.fr, lober@isir.upmc.fr
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
#include <ocraWbiPlugins/ocraWbiModel.h>

#include <modHelp/modHelp.h>
#include <iostream>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>

#include "taskSequences/sequenceLibrary.h"
#include "wocra/Tasks/wOcraTaskParser.h"


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
                                                             bool _runInDebugMode,
                                                             bool _isFreeBase)
    : RateThread(_period),
      name(_name),
      robotName(_robotName),
      robot(_wbi),
      options(_options),
      startupTaskSetPath(_startupTaskSetPath),
      startupSequence(_startupSequence),
      runInDebugMode(_runInDebugMode),
      processor(*this)
{
    // bool _isFreeBase = false;
    ocraModel = new ocraWbiModel(robotName, robot->getDoFs(), robot, _isFreeBase);
    bool useReducedProblem = false;
    ctrl = new wocra::wOcraController("icubControl", *ocraModel, internalSolver, useReducedProblem);

    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());

    homePosture = Eigen::VectorXd::Zero(robot->getDoFs());
    debugPosture = Eigen::VectorXd::Zero(robot->getDoFs());
    refSpeed = Eigen::VectorXd::Constant(robot->getDoFs(), 0.17);


    getHomePosture(*ocraModel, homePosture);
    getNominalPosture(*ocraModel, debugPosture);

    torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);


    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);

    fb_Hroot_Vector = yarp::sig::Vector(16, 0.0);
    fb_Troot_Vector = yarp::sig::Vector(6, 0.0);

    fb_torque.resize(robot->getDoFs());

    time_sim = 0;

    recorded = false;


}

ISIRWholeBodyControllerThread::~ISIRWholeBodyControllerThread()
{
    delete(taskSequence);
    // delete(ocraModel);
    // delete(ctrl);
    rpcPort.close();
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


    robot->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());

    if (runInDebugMode)
    {
        robot->setControlMode(CTRL_MODE_POS, debugPosture.data(), ALL_JOINTS);
        robot->setControlReference(debugPosture.data());
        robot->setControlMode(CTRL_MODE_TORQUE, 0, debugJointIndex);
    }
    else
    {
        // Set all declared joints in module to TORQUE mode
        bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    }


    /******************************************************************************************************
                                        Parse tasks and load sequence
    ******************************************************************************************************/




    if (runInDebugMode)
    {
        if (ocraModel->hasFixedRoot()) {
            std::cout << "Loading fixed base minimal tasks..." << std::endl;
            taskSequence = LoadSequence("Debug");
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
        std::cout << "\n\n=== Creating ISIRWholeBodyController ===" << std::endl;
        rpcPort.open("/ISIRWholeBodyController/rpc:i");
        rpcPort.setReader(processor);

        //Create cpp sequence
        if (!startupSequence.empty()) {
            std::cout << "\nLoading sequence:\n" << startupSequence << "\n" << std::endl;
            taskSequence = LoadSequence(startupSequence);
        }

        //Create XML task set
        if (!startupTaskSetPath.empty()) {
            if (startupSequence.empty()) {
                taskSequence = LoadSequence("Empty");
            }
            std::cout << "\nLoading tasks from XML file:\n" << startupTaskSetPath << "\n" << std::endl;
            wocra::wOcraTaskParser taskParser;
            if(taskParser.parseTasksXML( startupTaskSetPath.c_str() )){
                taskParser.addTaskManagersToSequence(*ctrl, *ocraModel, taskSequence);
            }
            else{
                //TODO: Implement fall pack procedure for failure to parse xml tasks.;
            }
        }
        else{
            std::cout << "No XML task set detected." << std::endl;
        }


        if ( (startupTaskSetPath.empty()) && (startupSequence.empty()) )
        {
            std::cout << "\nNo tasks or scenarios loaded on startup. Defaulting to standard initial tasks." << std::endl;
            if (!ocraModel->hasFixedRoot()) {
                std::cout << "Loading floating base minimal tasks..." << std::endl;
                taskSequence = LoadSequence("FloatingBaseMinimalTasks");
            }
            else{
                std::cout << "Loading fixed base minimal tasks..." << std::endl;
                taskSequence = LoadSequence("FixedBaseMinimalTasks");
            }
        }

    }



    /******************************************************************************************************
                                            Initialize sequence
    ******************************************************************************************************/


    taskSequence->init(*ctrl, *ocraModel);


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
            if (tempDebugIndex>=0 && tempDebugIndex<robot->getDoFs()) {
                std::cout << "\n-----\nNew joint received...\n" << std::endl;
                std::cout << "Returning joint: " << debugJointIndex << " to home position." << std::endl;
                robot->setControlMode(CTRL_MODE_POS, &debugPosture[debugJointIndex], debugJointIndex);
                robot->setControlReference(&debugPosture[debugJointIndex], debugJointIndex);

                debugJointIndex = tempDebugIndex;
                if(robot->setControlMode(CTRL_MODE_TORQUE, &torques_cmd[debugJointIndex], debugJointIndex) )
                {
                    std::cout << "Now joint: " << debugJointIndex << " is now being tested in torque control.\n-----\n" << std::endl;
                }
            }
            else{std::cout << "\n[WARNING] (thread.run) The command you sent was not a valid joint index. Please use integers between 0 and "<< robot->getDoFs() - 1 << ".\n"<< std::endl;}

        }
    }
    else{

        taskSequence->update(time_sim, *ocraModel, NULL);
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


     //record torque data
      double period = 6;
      double duration = 2*period;
      double ti = 0.2;
      if (time_sim>=ti && time_sim<ti+duration)
      {
          tau_square.push_back(eigenTorques.transpose()*eigenTorques);
      }
      else if (time_sim >= ti+duration && !recorded)
      {
          datafile.open ("./tau_data.txt");

          for (std::vector<double>::iterator it = tau_square.begin() ; it != tau_square.end(); ++it)
              datafile << *it <<" ";
          datafile<<"\n";
          datafile.close();
          recorded = true;
          std::cout<<"tau data saved."<<std::endl;
      }

    /******************************************************************************************************
                                    Send the torques to the robot via WBI
    ******************************************************************************************************/

    if (runInDebugMode) {
        robot->setControlReference(&torques_cmd[debugJointIndex], debugJointIndex);
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
//        else if (!ocraModel->hasFixedRoot()){
//            std::cout<< "\n---\nfb_Hroot:\n" << fb_Hroot_Vector(3) << " "<< fb_Hroot_Vector(7) << " "<< fb_Hroot_Vector(11) << std::endl;
//            // std::cout << "root_link pos\n" << ocraModel->getSegmentPosition(ocraModel->getSegmentIndex("root_link")).getTranslation().transpose() << std::endl;
//            std::cout<< "fb_Troot:\n" << fb_Troot_Vector(0) <<" "<< fb_Troot_Vector(1) <<" "<< fb_Troot_Vector(2) << "\n---\n";
//            std::cout << "root_link vel\n" << ocraModel->getSegmentVelocity(ocraModel->getSegmentIndex("root_link")).getLinearVelocity().transpose() << std::endl;
//        }

    }

    time_sim += TIME_MSEC_TO_SEC * getRate();
}

//*************************************************************************************************************************
void ISIRWholeBodyControllerThread::threadRelease()
{
    taskSequence->clearSequence();

    if(robot->setControlMode(CTRL_MODE_POS, 0, ALL_JOINTS) )
    {
        std::cout << "\n\n--> Closing controller thread. Switching to POSITION mode and returning to home pose.\n" << std::endl;

        if (ocraModel->hasFixedRoot()) {
            bool res_setControlReference = robot->setControlReference(homePosture.data());
        }
        else{
            bool res_setControlReference = robot->setControlReference(homePosture.data());
            //TODO: Implement a safe home park procedure for standing.
        }

    }
    else{
        std::cout << "[ERROR] (ISIRWholeBodyControllerThread::threadRelease): Could not set the robot into Position Control mode." << std::endl;
    }
}

/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
ISIRWholeBodyControllerThread::DataProcessor::DataProcessor(ISIRWholeBodyControllerThread& ctThreadRef):ctThread(ctThreadRef)
{
    //do nothing
}

bool ISIRWholeBodyControllerThread::DataProcessor::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;
    bool ok = input.read(connection);
    if (!ok)
        return false;

    else{
        ctThread.parseIncomingMessage(&input, &reply);
        yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/





void ISIRWholeBodyControllerThread::parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply)
{
    int btlSize = input->size();
    for (int i=0; i<btlSize;)
    {
        std::string msgTag = input->get(i).asString();

        if(msgTag == "addTaskYARP")
        {
            std::cout << "Implement yarp task parsing" << std::endl;
            i++;
        }

        // Stiffness
        else if(msgTag == "addTaskXML")
        {
            i++;
            wocra::wOcraTaskParser taskParser;
            taskParser.parseTasksXML( input->get(i).asString().c_str() );
            taskParser.addTaskManagersToSequence(*ctrl, *ocraModel, taskSequence);
            i++;
        }
        else if(msgTag == "removeTask")
        {
            i++;
            std::string taskToRemove = input->get(i).asString();
            taskSequence->removeTaskManager(taskToRemove);
            ctrl->removeTask(taskToRemove);
            reply->addString("Removed:");
            reply->addString(taskToRemove);
            i++;
        }

        else if(msgTag == "getTaskList")
        {
            reply->addString("tasks:");
            std::vector<std::string> strVector = taskSequence->getTaskList();
            for(int j=0; j<strVector.size(); j++){
                reply->addString(strVector[j]);
            }
            i++;
        }

        else if(msgTag == "getTaskPorts")
        {
            reply->addString("taskPorts:");
            std::vector<std::string> strVector = taskSequence->getTaskPorts();
            for(int j=0; j<strVector.size(); j++){
                reply->addString(strVector[j]);
            }
            i++;
        }

        else if (msgTag == "help")
        {
            // TODO: Properly print help message to rpc reply
            // reply->addString(printValidMessageTags());
            std::cout << printValidMessageTags();
            i++;
        }

        // Fallback
        else
        {
            std::cout << "[WARNING] (ISIRWholeBodyControllerThread::parseIncomingMessage): The message tag, " << msgTag << " doesn't exist. Skipping. Use help to see availible options." << std::endl;

            reply->addString("invalid_input");
            i++;
        }
    }
}

std::string ISIRWholeBodyControllerThread::printValidMessageTags()
{
    std::string helpString  = "\n=== Valid message tags are: ===\n";
    helpString += "removeTask: Allows you to remove a single task manager from the sequence.\n";
    return helpString;
}
