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

#include <gOcraController/thread.h>
#include <../../gOcraController/include/gOcraController/ocraWbiModel.h>
#include <modHelp/modHelp.h>
#include <iostream>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


//#include "gocra/Solvers/OneLevelSolver.h"
#include "gocra/Features/gOcraFeature.h"
#include "ocra/control/Feature.h"
#include "ocra/control/FullState.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/control/ControlEnum.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <algorithm>

using namespace gOcraController;
using namespace yarp::math;
using namespace yarpWbi;
using namespace std;

#define ALL_JOINTS -1
#define DIM_DISP 3
#define DIM_TWIST 6
#define TORQUE_MIN -24
#define TORQUE_MAX 24
#define TIME_MSEC_TO_SEC 0.001

void getNPosture(gocra::gOcraModel &model, VectorXd &q);

//*************************************************************************************************************************
gOcraControllerThread::gOcraControllerThread(string _name,
                                             string _robotName,
                                             int _period,
                                             wholeBodyInterface *_wbi,
                                             yarp::os::Property &_options,
                                             std::string _replayJointAnglesPath
                                            )
    : RateThread(_period), name(_name), robotName(_robotName), robot(_wbi), options(_options), replayJointAnglesPath(_replayJointAnglesPath)
{
//    if(!replayJointAnglesPath.empty()){
//      std::cout << "Got the replay flag - replaying joint angles in position mode." << std::endl;
//      std::cout << "Getting joint angles from file:\n" << replayJointAnglesPath << std::endl;
//      //TODO: Parse file path and load joint commands into a big ass matrix
//      //TODO: Pre shape a "PosCommandVector" which will pick the angles at each timestep - this will get passed to the WBI
//      isReplayMode = true;
//    }else{
//      isReplayMode = false;
//    }


    bool isFreeBase = false;
    ocraModel = new ocraWbiModel(robotName, robot->getDoFs(), robot, isFreeBase);
    bool useGrav = true;// enable gravity compensation
    ctrl = new gocra::GHCJTController("icubControl", *ocraModel, internalSolver, useGrav);

    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());

    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);

    fb_Hroot_Vector = yarp::sig::Vector(16, 0.0);
    fb_Troot_Vector = yarp::sig::Vector(6, 0.0);

    fb_torque.resize(robot->getDoFs());

//    position_cmd.resize(robot->getDoFs());
    position_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);

    time_sim = 0;


}

//*************************************************************************************************************************
bool gOcraControllerThread::threadInit()
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




    //================ SET UP TASK AND CONTROL MODE===================//
    FILE *infile = fopen("/home/codyco/icub/software/src/codyco-superbuild/main/ocra-wbi-plugins/modules/gOcraController/sequence_param.txt", "r");
    char keyward[256];
    char value[128];
    int seq;
    char positionControl[128];
    char qdFilePath[512];
    char dqdFilePath[512];
    double kp,kd,ki;

    while (fgets(keyward, sizeof(keyward), infile)){
        if (1 == sscanf(keyward, "sequence = %127s", value)){
            seq = atoi(value);
        }
        sscanf(keyward, "positionControl = %127s", positionControl);
        sscanf(keyward, "qdFilePath = %127s", qdFilePath);
        sscanf(keyward, "dqdFilePath = %127s", dqdFilePath);
        if (1 == sscanf(keyward, "kp = %127s", value)){
            kp = atof(value);
            std::cout << "kp = " << kp << std::endl;
        }
        if (1 == sscanf(keyward, "kd = %127s", value)){
            kd = atof(value);
            std::cout << "kd = " << kd << std::endl;
        }
        if (1 == sscanf(keyward, "ki = %127s", value)){
            ki = atof(value);
            std::cout << "ki = " << ki << std::endl;
        }

    }
    std::cout<<"qdFilePath ="<< qdFilePath<<std::endl;
    std::cout<<"dqdFilePath ="<< dqdFilePath<<std::endl;

    // Set control mode
    if (strcmp(positionControl, "true")==0){
        isReplayMode=true;
        std::cout<<"position control mode"<<std::endl;
    }
    else{
         isReplayMode = false;
         std::cout<<"torque control mode"<<std::endl;
    }

    if (!isReplayMode) {
      bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    }
    else{
//      bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, 0, ALL_JOINTS);
      bool res_setControlMode = robot->setControlMode(CTRL_MODE_DIRECT_POSITION, 0, ALL_JOINTS);

      // read reference motion from files
      ifstream jointPositionFile(qdFilePath);
      ifstream jointVelocityFile(dqdFilePath);
      unsigned int positionFileLength = 0;
      positionFileLength = std::count(std::istreambuf_iterator<char>(jointPositionFile), std::istreambuf_iterator<char>(), '\n');

      unsigned int velocityFileLength = 0;
      velocityFileLength = std::count(std::istreambuf_iterator<char>(jointVelocityFile), std::istreambuf_iterator<char>(), '\n');

//      replayDataLength = positionFileLength>velocityFileLength?velocityFileLength:positionFileLength;
      replayDataLength = 6000;
      std::cout<<"replayDataLength="<<replayDataLength<<std::endl;

      jointPositionFile.clear();
      jointPositionFile.seekg(0, jointPositionFile.beg);
      jointVelocityFile.clear();
      jointVelocityFile.seekg(0, jointVelocityFile.beg);

      qRef = Eigen::MatrixXd::Zero(replayDataLength, robot->getDoFs());
      dqRef = Eigen::MatrixXd::Zero(replayDataLength, robot->getDoFs());


      int dofmax = 25;
      double tmp;
      for (int rowIndex=0; rowIndex<replayDataLength; ++rowIndex){
          if (robot->getDoFs()<25){ // when left leg is inactive
              for (int colIndex=0; colIndex<dofmax; ++colIndex){
                  if (colIndex<=12){
                      jointPositionFile >> qRef(rowIndex,colIndex);
                      jointVelocityFile >> dqRef(rowIndex,colIndex);
                  }
                  else if (colIndex>12 && colIndex <19){//left leg DOFs
                      jointPositionFile >> tmp;
                      jointVelocityFile >> tmp;
                  }
                  else{
                      jointPositionFile >> qRef(rowIndex,colIndex-6);
                      jointVelocityFile >> dqRef(rowIndex,colIndex-6);
                  }

              }
          }
          else{ // use whole body DOF
              for (int colIndex=0; colIndex<robot->getDoFs(); ++colIndex){
                      jointPositionFile >> qRef(rowIndex,colIndex);
                      jointVelocityFile >> dqRef(rowIndex,colIndex);
              }
          }

      }

      jointPositionFile.close();
      jointVelocityFile.close();
    }


    // set task sequence
    if (!isReplayMode) {
        if (seq==1){
            std::cout << "Sequence_NominalPose" << std::endl;
            sequence = new Sequence_NominalPose();
        }
        else if (seq==2){
            std::cout << "Sequence_InitialPoseHold" << std::endl;
            sequence = new Sequence_InitialPoseHold();
        }
        else if (seq==3){
            std::cout << "Sequence_LeftHandReach" << std::endl;
            sequence = new Sequence_LeftHandReach();
        }
        else if (seq==4){
            std::cout << "Sequence_ComLeftHandReach" << std::endl;
            sequence = new Sequence_ComLeftHandReach();
        }
        else{
            std::cout << "Sequence_NominalPose" << std::endl;
            sequence = new Sequence_NominalPose();
        }

    }
    else{
        if (seq==4){
            std::cout << "Sequence_ComLeftHandReachReplay" << std::endl;
            sequence = new Sequence_ComLeftHandReachReplay();
        }
        else{
            std::cout << "Sequence_NominalPose" << std::endl;
            sequence = new Sequence_NominalPose();
        }
    }


    sequence->init(*ctrl, *ocraModel);


    counter = 0;



	return true;
}

//*************************************************************************************************************************
void gOcraControllerThread::run()
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

        ocraModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    }
    else
        ocraModel->setState(fb_qRad, fb_qdRad);

    sequence->update(time_sim, *ocraModel, NULL);


    // TORQUE CONTROL MODE: compute desired torque by calling the controller and send command
    Eigen::VectorXd eigenTorques = Eigen::VectorXd::Constant(ocraModel->nbInternalDofs(), 0.0);
    if (!isReplayMode) {
        ctrl->computeOutput(eigenTorques);
        for(int i = 0; i < eigenTorques.size(); ++i)
        {
          if(eigenTorques(i) < TORQUE_MIN) eigenTorques(i) = TORQUE_MIN;
          else if(eigenTorques(i) > TORQUE_MAX) eigenTorques(i) = TORQUE_MAX;
        }

        modHelp::eigenToYarpVector(eigenTorques, torques_cmd);
        robot->setControlReference(torques_cmd.data());
    }

    // verify joint limits
    Eigen::VectorXd q_margin = 0.001*Eigen::VectorXd::Ones(robot->getDoFs());
    Eigen::VectorXd q_min = ocraModel->getJointLowerLimits() + q_margin;
    Eigen::VectorXd q_max = ocraModel->getJointUpperLimits() - q_margin;
    Eigen::VectorXd q_actual = ocraModel->getJointPositions();
    Eigen::VectorXd q_vel = ocraModel->getJointVelocities();
//    Eigen::VectorXd q_max_actual = q_max - q_actual;
//    Eigen::VectorXd q_actual_min = q_actual - q_min;

//    if (!((q_actual.array()<=(q_max-q_margin).array()).all() && (q_actual.array()>=(q_min+q_margin).array()).all())){
//        std::cout<<"JL! ";
//    }
//    wbi::ID dofID;

//    for (int i =0; i<robot->getDoFs(); ++i){
//            robot->getJointList().indexToID(i, dofID);
//            std::cout<<i<<": "<<dofID.toString()<<std::endl;
//    }

//    if (!((q_actual.array()<=(q_max-q_margin).array()).all()))
//        std::cout<<std::endl<<"UL";
//    for (int i =0; i<robot->getDoFs(); ++i){
//        if (q_actual(i)>q_max(i)){
//            robot->getJointList().indexToID(i, dofID);
//            std::cout<<dofID.toString()<<"";
//        }
//    }

//    if (!(q_actual.array()>=(q_min+q_margin).array()).all())
//        std::cout<<std::endl<<"LL";
//    for (int i =0; i<robot->getDoFs(); ++i){
//        if (q_actual(i)<q_min(i)){
//            robot->getJointList().indexToID(i, dofID);
//            std::cout<<dofID.toString()<<"";
//        }
//    }



    // POSITION CONTROL MODE: get reference motion and send command
    if (isReplayMode) {
        double dt = TIME_MSEC_TO_SEC * getRate();
//        Eigen::VectorXd eigenJointPosition = Eigen::VectorXd::Constant(ocraModel->nbInternalDofs(), 0.0);
//        Eigen::VectorXd generalizedForces = Eigen::VectorXd::Constant(ocraModel->nbInternalDofs(), 0.0);
//        generalizedForces = ocraModel->getGravityTerms() + ocraModel->getNonLinearTerms() + eigenTorques;
////        std::cout<<"gforces: "<<generalizedForces.transpose()<<std::endl;
//        eigenJointPosition = q_actual + q_vel*dt + 0.5*ocraModel->getInertiaMatrixInverse()*generalizedForces*dt*dt;
////        std::cout<<"position: "<<eigenJointPosition.transpose()<<std::endl;
//        for (int i =0; i<robot->getDoFs(); ++i){
//            if (eigenJointPosition(i)>q_max(i))
//                eigenJointPosition(i)=q_max(i);

//             if (eigenJointPosition(i)<q_min(i))
//                eigenJointPosition(i)=q_min(i);
//        }
//        modHelp::eigenToYarpVector(q_actual, position_cmd);
        int dof = robot->getDoFs();
//        Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(dof);
//        getNPosture(*ocraModel, nominal_q);

        yarp::sig::Vector refSpeed(dof, 0.0);
        yarp::sig::Vector refAcc(dof, 0.0);
//        yarp::sig::Vector pos(dof, 0.0);
//         modHelp::eigenToYarpVector(q_actual, pos);
//         modHelp::eigenToYarpVector(nominal_q, position_cmd);
//        for (int i=0; i<robot->getDoFs(); ++i){
//            refSpeed[i] = 0.001*(position_cmd[i]-pos[i])/dt;
//        }

        Eigen::VectorXd refPos = Eigen::VectorXd::Zero(dof);
        Eigen::VectorXd refVel = Eigen::VectorXd::Zero(dof);
        if (counter<replayDataLength){
            refPos = qRef.block(counter,0,1,dof).transpose();
            refVel = dqRef.block(counter,0,1,dof).transpose();

            for (int i =0; i<robot->getDoFs(); ++i){
                if (refPos(i)>q_max(i))
                    refPos(i)=q_max(i);

                 if (refPos(i)<q_min(i))
                    refPos(i)=q_min(i);
            }
            modHelp::eigenToYarpVector(refPos, position_cmd);
            modHelp::eigenToYarpVector(refVel, refSpeed);
        }

//        robot->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
//        robot->setControlParam(CTRL_PARAM_REF_ACC, refAcc.data(), ALL_JOINTS);


//        position_cmd = yarp::sig::Vector(robot->getDoFs(), 0.1);
        robot->setControlReference(position_cmd.data());
//        robot->setControlMode(CTRL_MODE_POS, position_cmd.data(), ALL_JOINTS);

        if (counter<10000)
            counter++;
    }

    time_sim += TIME_MSEC_TO_SEC * getRate();

}

//*************************************************************************************************************************
void gOcraControllerThread::threadRelease()
{
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, 0, ALL_JOINTS);

//    bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, q_initial.data(), ALL_JOINTS);

    //yarp::sig::Vector torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);
    //robot->setControlReference(torques_cmd.data());
}

void getNPosture(gocra::gOcraModel& model, VectorXd &q)
{
    q[model.getDofIndex("torso_pitch")] = M_PI / 18;
    q[model.getDofIndex("r_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("r_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("l_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("r_shoulder_pitch")] = -M_PI / 6;    
//    q[model.getDofIndex("l_hip_pitch")] = M_PI / 8;
//    q[model.getDofIndex("l_hip_roll")] = M_PI / 18;
//    q[model.getDofIndex("l_knee")] = -M_PI / 6;
    q[model.getDofIndex("r_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("r_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("r_knee")] = -M_PI / 6;
}


