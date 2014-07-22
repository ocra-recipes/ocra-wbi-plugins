
/*
 * Copyright (C) 2013 CODYCO Project
 * Author: Serena Ivaldi, Joseph Salini 
 * email:  serena.ivaldi@isir.upmc.fr, joseph.salini@isir.upmc.fr
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 
A simple test of integration between ORCISIR and YARP.

This is an integration example between icubSim and the iCub model with fixed base (icubfixed.h) used by ORCISIR, ISIR-XDE etcetera.
  
 \section tested_os_sec Tested OS
 Linux
 
 \author Serena Ivaldi, Joseph Salini
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <modHelp/modHelp.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <iostream>
#include <Eigen/Eigen>
// the model used to test ORCISIR
//#include "icubfixed.h"
#include "iCubModel.h"
// these are to include ORCISIR
#include "orcisir/ISIRController.h"
#include "orcisir/Solvers/OneLevelSolver.h"

#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"

#include "orcisir/Features/ISIRFeature.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
//using namespace modHelp;
using namespace iCub::ctrl;
using namespace std;

// necessary for cartesian interfaces
YARP_DECLARE_DEVICES(icubmod)


Vector evalVel(const Vector &x, AWLinEstimator  *linEst)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return linEst->estimate(el);  
}

Vector evalAcc(const Vector &x, AWQuadEstimator *quadEstLow)
{
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    
    return quadEstLow->estimate(el);
}

void JointsToAll(Vector &q_all, 
    Vector &encoders_torso, Vector &encoders_head, Vector &encoders_arm_left,
    Vector &encoders_arm_right, Vector &encoders_leg_left, Vector &encoders_leg_right)
{
    //encoders returns degrees
    if(q_all.size()!=32) q_all.resize(32,0.0);

    //q_all[0] = 0;                       //icub.waist (the base moving in the 6DOF space)
    q_all[0] = encoders_leg_left[0]*M_PI/180;    //icub.l_hip_1
    q_all[1] = encoders_leg_left[1]*M_PI/180;    //icub.l_hip_2
    q_all[2] = encoders_leg_left[2]*M_PI/180;    //icub.l_thigh
    q_all[3] = encoders_leg_left[3]*M_PI/180;    //icub.l_shank
    q_all[4] = encoders_leg_left[4]*M_PI/180;    //icub.l_ankle_1
    q_all[5] = encoders_leg_left[5]*M_PI/180;    //icub.l_foot
    q_all[6] = encoders_torso[0]*M_PI/180;       //icub.lap_belt_1
    q_all[7] = encoders_torso[1]*M_PI/180;       //icub.lap_belt_2
    q_all[8] = encoders_torso[2]*M_PI/180;       //icub.chest
    q_all[9]= encoders_arm_right[0]*M_PI/180;   //icub.r_shoulder_1
    q_all[10]= encoders_arm_right[1]*M_PI/180;   //icub.r_shoulder_2
    q_all[11]= encoders_arm_right[2]*M_PI/180;   //icub.r_arm
    q_all[12]= encoders_arm_right[3]*M_PI/180;   //icub.r_elbow_1
    q_all[13]= encoders_arm_right[4]*M_PI/180;   //icub.r_forearm
    q_all[14]= encoders_arm_right[5]*M_PI/180;   //icub.r_wrist_1
    q_all[15]= encoders_arm_right[6]*M_PI/180;   //icub.r_hand
    q_all[16]= encoders_arm_left[0]*M_PI/180;    //icub.l_shoulder_1
    q_all[17]= encoders_arm_left[1]*M_PI/180;    //icub.l_shoulder_2
    q_all[18]= encoders_arm_left[2]*M_PI/180;    //icub.l_arm
    q_all[19]= encoders_arm_left[3]*M_PI/180;    //icub.l_elbow_1
    q_all[20]= encoders_arm_left[4]*M_PI/180;    //icub.l_forearm
    q_all[21]= encoders_arm_left[5]*M_PI/180;    //icub.l_wrist_1
    q_all[22]= encoders_arm_left[6]*M_PI/180;    //icub.l_hand
    q_all[23]= encoders_head[0]*M_PI/180;        //icub.neck_1
    q_all[24]= encoders_head[1]*M_PI/180;        //icub.neck_2
    q_all[25]= encoders_head[2]*M_PI/180;        //icub.head
    q_all[26]= encoders_leg_right[0]*M_PI/180;   //icub.r_hip_1
    q_all[27]= encoders_leg_right[1]*M_PI/180;   //icub.r_hip_2
    q_all[28]= encoders_leg_right[2]*M_PI/180;   //icub.r_thigh
    q_all[29]= encoders_leg_right[3]*M_PI/180;   //icub.r_shank
    q_all[30]= encoders_leg_right[4]*M_PI/180;   //icub.r_ankle_1
    q_all[31]= encoders_leg_right[5]*M_PI/180;   //icub.r_foot

}

void filterVelAcc_All(Vector &q_all, Vector &dq_all, Vector &ddq_all, 
    AWLinEstimator  *velEst, AWQuadEstimator *accEst)
{
    dq_all =  evalVel(q_all,velEst);
    ddq_all = evalAcc(q_all,accEst);
}




//===============================
//===============================

//        MAIN

//===============================
//===============================

int main(int argc, char** argv)
{
    YARP_REGISTER_DEVICES(icubmod)

    // first YARP
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }

    // stuff related to connection with iCubSim
    string robot="icubGazeboSim";
    string name="icub";


    //================================
    //================================

    //        DATA VARIABLES

    //================================
    //================================


    // drivers properties
    yarp::os::Property optionsRA, optionsLA,optionsTO, optionsHE, optionsLL, optionsRL; 
    yarp::os::Property optionsCartLA;
    // drivers icub parts
    yarp::dev::PolyDriver *ddRA=NULL, *ddLA=NULL;
    yarp::dev::PolyDriver *ddTO=NULL, *ddHE=NULL;
    yarp::dev::PolyDriver *ddRL=NULL, *ddLL=NULL;
    yarp::dev::PolyDriver *ddTorqueLA=NULL, *ddTorqueRA=NULL;
    yarp::dev::PolyDriver *ddTorqueLL=NULL, *ddTorqueRL=NULL;
    yarp::dev::PolyDriver *ddTorqueHE=NULL, *ddTorqueTO=NULL;
    // drivers cartesian
    yarp::dev::PolyDriver *ddCartLA=NULL;
    // motor interfaces
    IPositionControl *iposRA=NULL, *iposLA=NULL,*ipos=NULL;
    IPositionControl *iposRL=NULL, *iposLL=NULL;
    IPositionControl *iposTO=NULL, *iposHE=NULL;
    IVelocityControl *ivelRA=NULL, *ivelLA=NULL,*ivel=NULL;
    IVelocityControl *ivelRL=NULL, *ivelLL=NULL;
    IVelocityControl *ivelTO=NULL, *ivelH=NULL;
    yarp::dev::ITorqueControl *itorqueLA=NULL, *itorqueRA=NULL;
    yarp::dev::ITorqueControl *itorqueLL=NULL, *itorqueRL=NULL;
    yarp::dev::ITorqueControl *itorqueHE=NULL, *itorqueTO=NULL;
    yarp::dev::IControlMode *controlModeLA=NULL, *controlModeRA=NULL;
    yarp::dev::IControlMode *controlModeLL=NULL, *controlModeRL=NULL;
    yarp::dev::IControlMode *controlModeHE=NULL, *controlModeTO=NULL;
    yarp::os::Property optionsTorqueLA,optionsTorqueRA;
    yarp::os::Property optionsTorqueLL,optionsTorqueRL;
    yarp::os::Property optionsTorqueHE,optionsTorqueTO;
    IEncoders *iencRA=NULL, *iencLA=NULL;
    IEncoders *iencRL=NULL, *iencLL=NULL;
    IEncoders *iencTO=NULL, *iencHE=NULL;
    ICartesianControl *icrtLA=NULL;
    ICartesianControl *icrtRA=NULL;
    //
    int startup_context_id_LA;
    //
    Vector encoders_arm_left(7,0.0);
    Vector encoders_arm_right(7,0.0);
    Vector encoders_head(3,0.0);
    Vector encoders_leg_left(6,0.0);
    Vector encoders_leg_right(6,0.0);
    Vector encoders_torso(3,0.0);

    Vector q_all(32,0.0), dq_all(32,0.0), ddq_all(32,0.0);
    dq_all = ddq_all = q_all;
    AWLinEstimator  *velEst = new AWLinEstimator(16,1.0); 
    AWQuadEstimator *accEst = new AWQuadEstimator(25,1.0);

    //================================
    //================================

    //      CONNECT TO ICUB

    //================================
    //================================


    // opening drivers
    
    // right arm
    optionsRA.put("device","remote_controlboard");
    optionsRA.put("local",string("/"+name+"/right_arm").c_str());
    optionsRA.put("remote",string("/"+robot+"/right_arm").c_str());
    // left arm
    optionsLA.put("device","remote_controlboard");
    optionsLA.put("local",string("/"+name+"/left_arm").c_str());
    optionsLA.put("remote",string("/"+robot+"/left_arm").c_str());
    // right leg
    optionsRL.put("device","remote_controlboard");
    optionsRL.put("local",string("/"+name+"/right_leg").c_str());
    optionsRL.put("remote",string("/"+robot+"/right_leg").c_str());
    // left leg
    optionsLL.put("device","remote_controlboard");
    optionsLL.put("local",string("/"+name+"/left_leg").c_str());
    optionsLL.put("remote",string("/"+robot+"/left_leg").c_str());
    // torso
    optionsTO.put("device","remote_controlboard");
    optionsTO.put("local",string("/"+name+"/torso").c_str());
    optionsTO.put("remote",string("/"+robot+"/torso").c_str());
    // head
    optionsHE.put("device","remote_controlboard");
    optionsHE.put("local",string("/"+name+"/head").c_str());
    optionsHE.put("remote",string("/"+robot+"/head").c_str());
    //

    // torque left arm
    optionsTorqueLA.put("device","remote_controlboard");
    optionsTorqueLA.put("local",string("/torqueLA/left_arm").c_str());
    optionsTorqueLA.put("remote",string("/"+robot+"/left_arm").c_str());
    // torque right arm
    optionsTorqueRA.put("device","remote_controlboard");
    optionsTorqueRA.put("local",string("/torqueRA/right_arm").c_str());
    optionsTorqueRA.put("remote",string("/"+robot+"/right_arm").c_str());
    // torque head
    optionsTorqueHE.put("device","remote_controlboard");
    optionsTorqueHE.put("local",string("/torqueHE/head").c_str());
    optionsTorqueHE.put("remote",string("/"+robot+"/head").c_str());
    // torque left leg
    optionsTorqueLL.put("device","remote_controlboard");
    optionsTorqueLL.put("local",string("/torqueLL/left_leg").c_str());
    optionsTorqueLL.put("remote",string("/"+robot+"/left_leg").c_str());
    // torque right leg
    optionsTorqueRL.put("device","remote_controlboard");
    optionsTorqueRL.put("local",string("/torqueRL/right_leg").c_str());
    optionsTorqueRL.put("remote",string("/"+robot+"/right_leg").c_str());
    // torque torso
    optionsTorqueTO.put("device","remote_controlboard");
    optionsTorqueTO.put("local",string("/torqueTO/torso").c_str());
    optionsTorqueTO.put("remote",string("/"+robot+"/torso").c_str());

    // torso
    //....................................................
    if(!modHelp::createDriver(ddTO, optionsTO))
    {
        cout<<"Error: unable to create driver for torso"<<endl;
        ddTO->close();
        return false;
    }
    if(!ddTO->view(iencTO) || !ddTO->view(iposTO) || !ddTO->view(ivelTO))
    {
        cout<<"Problems acquiring interfaces of torso"<<endl;
        ddTO->close();
        return false;
    }
    // init torso
    int torsoAxes;
    iencTO->getAxes(&torsoAxes);
    Vector torso;
    torso.resize(torsoAxes,0.0);
    
    // head
    //....................................................
    if(!modHelp::createDriver(ddHE,optionsHE))
    {
        cout<<"Error: unable to create driver of head"<<endl;
        ddHE->close();
        return false;
    }
    if(!ddHE->view(iencHE) || !ddHE->view(iposHE) )
    {
        cout<<"Error: problems acquiring interfaces of head"<<endl;
        ddHE->close();
        return false;
    }
    
    // init head
    int headAxes;
    iencHE->getAxes(&headAxes);
    Vector head;
    head.resize(headAxes,0.0);
    
    // right arm
    //....................................................
    if(!modHelp::createDriver(ddRA,optionsRA))
    {
        cout<<"Problems connecting to the remote driver of right_arm"<<endl;
        ddRA->close();
        return false;
    }
    if(!ddRA->view(iencRA) || !ddRA->view(iposRA) )
    {
        cout<<"Problems acquiring interfaces of right_arm"<<endl;
        ddRA->close();
        return false;
    }
    
    // left arm
    //....................................................
    
    if(!modHelp::createDriver(ddLA,optionsLA))
    {
        cout<<"Problems connecting to the remote driver of left_arm"<<endl;
        ddLA->close();
        return false;
    }
    if(!ddLA->view(iencLA) || !ddLA->view(iposLA) )
    {
        cout<<"Problems acquiring interfaces of left_arm"<<endl;
        ddLA->close();
        return false;
    }
    
    // right leg
    //....................................................
    if(!modHelp::createDriver(ddRL,optionsRL))
    {
        cout<<"Problems connecting to the remote driver of right_leg"<<endl;
        ddRL->close();
        return false;
    }
    if(!ddRL->view(iencRL) || !ddRL->view(iposRL) )
    {
        cout<<"Problems acquiring interfaces of right_leg"<<endl;
        ddRL->close();
        return false;
    }
    
    // left leg
    //....................................................
    if(!modHelp::createDriver(ddLL,optionsLL))
    {
        cout<<"Problems connecting to the remote driver of left_leg"<<endl;
        ddLL->close();
        return false;
    }
    if(!ddLL->view(iencLL) || !ddLL->view(iposLL) )
    {
        cout<<"Problems acquiring interfaces of left_leg"<<endl;
        ddLL->close();
        return false;
    }

    //Torque LA
    if(!modHelp::createDriver(ddTorqueLA,optionsTorqueLA))
    {
        cout<<"Problems connecting to the remote torque driver of left_arm"<<endl;
        ddTorqueLA->close();
        return false;
    }
    if(!ddTorqueLA->view(itorqueLA) || !ddLA->view(controlModeLA) )
    {
        cout<<"Problems acquiring interfaces of left_arm"<<endl;
        ddTorqueLA->close();
        return false;
    }

    //Torque RA
    if(!modHelp::createDriver(ddTorqueRA,optionsTorqueRA))
    {
        cout<<"Problems connecting to the remote torque driver of right_arm"<<endl;
        ddTorqueRA->close();
        return false;
    }
    if(!ddTorqueRA->view(itorqueRA) || !ddRA->view(controlModeRA) )
    {
        cout<<"Problems acquiring interfaces of right_arm"<<endl;
        ddTorqueRA->close();
        return false;
    }

    //Torque HE
    if(!modHelp::createDriver(ddTorqueHE,optionsTorqueHE))
    {
        cout<<"Problems connecting to the remote torque driver of head"<<endl;
        ddTorqueHE->close();
        return false;
    }
    if(!ddTorqueHE->view(itorqueHE) || !ddHE->view(controlModeHE) )
    {
        cout<<"Problems acquiring interfaces of head"<<endl;
        ddTorqueHE->close();
        return false;
    }

    //Torque LL
    if(!modHelp::createDriver(ddTorqueLL,optionsTorqueLL))
    {
        cout<<"Problems connecting to the remote torque driver of left_leg"<<endl;
        ddTorqueLL->close();
        return false;
    }
    if(!ddTorqueLL->view(itorqueLL) || !ddLL->view(controlModeLL) )
    {
        cout<<"Problems acquiring interfaces of left_leg"<<endl;
        ddTorqueLL->close();
        return false;
    }

    //Torque RL
    if(!modHelp::createDriver(ddTorqueRL,optionsTorqueRL))
    {
        cout<<"Problems connecting to the remote torque driver of right_leg"<<endl;
        ddTorqueRL->close();
        return false;
    }
    if(!ddTorqueRL->view(itorqueRL) || !ddRL->view(controlModeRL) )
    {
        cout<<"Problems acquiring interfaces of right_leg"<<endl;
        ddTorqueRL->close();
        return false;
    }

    //Torque TO
    if(!modHelp::createDriver(ddTorqueTO,optionsTorqueTO))
    {
        cout<<"Problems connecting to the remote torque driver of torso"<<endl;
        ddTorqueTO->close();
        return false;
    }
    if(!ddTorqueTO->view(itorqueTO) || !ddTO->view(controlModeTO) )
    {
        cout<<"Problems acquiring interfaces of torso"<<endl;
        ddTorqueTO->close();
        return false;
    }

    std::cout << "Setting TorqueMode" << std::endl;
    //controlModeLA->setTorqueMode(0);
    controlModeLA->setTorqueMode(1);
    controlModeRA->setTorqueMode(1);
    controlModeHE->setTorqueMode(1);
    controlModeLL->setTorqueMode(1);
    controlModeRL->setTorqueMode(1);
    controlModeTO->setTorqueMode(1);

     cout<<"\n\nAll iCub parts where reached successfully."<<endl;
     cout<<"\n\nNow configuring the task."<<endl;
     Time::delay(2);


    //================================
    //================================

    //           ORCISIR

    //================================
    //================================



    //---------------------------------------------------------------------------------
    // now stuff related to the initialization of ORCISIR and its model

    // parameters for ORC
    bool useReducedProblem = false;

    // vectors for ORC
    VectorXd q(32);  
    VectorXd dq(32);
    VectorXd ddq(32);
    VectorXd tau(32); 
    double dt = 0.005;
   
    //SET CONTROLLER PARAMETERS
    cout<<"SET PARAMETERS\n";
    orcisir::OneLevelSolverWithQuadProg   internalSolver;


    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    cout<<"INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER\n";
    iCubModel* model;
    model = new iCubModel("icub");
    orcisir::ISIRController               ctrl("myCtrl", *model, internalSolver, useReducedProblem);

    // in iCubModel we have N joints = 32 + 1 DOF for to anchor the base
    q      = Eigen::VectorXd::Constant(model->nbInternalDofs(), 0.1);
    dq     = Eigen::VectorXd::Zero(model->nbInternalDofs());
    tau    = Eigen::VectorXd::Zero(model->nbInternalDofs());


    //CREATE SOME TASKS
    cout<<"CREATE TASKS\n";
//================ FULL STATE ==============================================================================//
    orc::FullModelState*   FMS;
    FMS = new orc::FullModelState("torqueTask.FModelState", *model, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState*  FTS;
    FTS = new orc::FullTargetState("torqueTask.FTargetState", *model, orc::FullState::INTERNAL);
    orc::FullStateFeature* feat;
    feat = new orc::FullStateFeature("torqueTask", *FMS);
    orc::FullStateFeature* featDes;
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);

    FTS->set_q(Eigen::VectorXd::Constant(model->nbInternalDofs(), 0.0));


    orcisir::ISIRTask* accTask;
    accTask = &(ctrl.createISIRTask("accTask", *feat, *featDes));
    accTask->initAsAccelerationTask();
    ctrl.addTask(*accTask);
    accTask->activateAsObjective();
    accTask->setStiffness(100);
    accTask->setDamping(4);
    accTask->setWeight(0.01);

    // task for left hand of icub
    // the segment hosting the frame must be indicated (it is defined in icubfixed.cpp)
    // frameTask is a Cartesian task
//================ FRAME ==============================================================================//
    orc::SegmentFrame*        SF;
    SF = new orc::SegmentFrame("frame.SFrame", *model, "l_hand", Eigen::Displacementd());
    orc::TargetFrame*         TF;
    TF = new orc::TargetFrame("frame.TFrame", *model);
    orc::PositionFeature* feat2;
    feat2 = new orc::PositionFeature("frame", *SF, orc::XYZ);
    orc::PositionFeature* featDes2;
    featDes2 = new orc::PositionFeature("frame.Des", *TF, orc::XYZ);

    TF->setPosition(Eigen::Displacementd(-0.3,-0.3,0.2));
    TF->setVelocity(Eigen::Twistd());
    TF->setAcceleration(Eigen::Twistd());

    orcisir::ISIRTask* accTask2;
    accTask2 = &(ctrl.createISIRTask("accTask2", *feat2, *featDes2));
    accTask2->initAsAccelerationTask();
    ctrl.addTask(*accTask2);
    accTask2->activateAsObjective();
    accTask2->setStiffness(200);//2000000000
    accTask2->setDamping(80);
    accTask2->setWeight(100.0);


    //---------------------------------------------------------------------------------
    // init simulation

    // take q and qdot from icub simulator (it has its own dynamics)
    iencLA->getEncoders(encoders_arm_left.data());
    iencRA->getEncoders(encoders_arm_right.data());
    iencHE->getEncoders(encoders_head.data());
    iencLL->getEncoders(encoders_leg_left.data());
    iencRL->getEncoders(encoders_leg_right.data());
    iencTO->getEncoders(encoders_torso.data());
 
    // conversion q: from icub parts to a big q vector for icub model
    JointsToAll(q_all, 
    encoders_torso, encoders_head, encoders_arm_left,
    encoders_arm_right, encoders_leg_left, encoders_leg_right);

    // filter to estimate qdot
    filterVelAcc_All(q_all,dq_all, ddq_all, velEst, accEst);

    //convert from yarp to eigen
    // look eigenToYarpVector
    modHelp::yarpToEigenVector(q_all, q);
    modHelp::yarpToEigenVector(dq_all,dq);
    modHelp::yarpToEigenVector(ddq_all,ddq);

    //set q_all inside icub model
    std::cout << "Size " << q.size() << " "  << q.transpose() << std::endl;
    model->setJointPositions(q); // set joints values on icub model fixed
    model->setJointVelocities(dq); // set also velocities


    //---------------------------------------------------------------------------------
    // now simulation with ORCISIR and its model and connection with iCubSim

    //SIMULATE
    cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {

        // is it necessary? yes...
        Time::delay(dt);

        cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";
        // take q and qdot from icub simulator (it has its own dynamics)
        iencLA->getEncoders(encoders_arm_left.data());
        iencRA->getEncoders(encoders_arm_right.data());
        iencHE->getEncoders(encoders_head.data());
        iencLL->getEncoders(encoders_leg_left.data());
        iencRL->getEncoders(encoders_leg_right.data());
        iencTO->getEncoders(encoders_torso.data());
        // conversion q: from icub parts to a big q vector for icub model
        JointsToAll(q_all, 
        encoders_torso, encoders_head, encoders_arm_left,
        encoders_arm_right, encoders_leg_left, encoders_leg_right);

        // filter to estimate qdot
        filterVelAcc_All(q_all,dq_all, ddq_all, velEst, accEst);

        //convert from yarp to eigen
        // look eigenToYarpVector
        modHelp::yarpToEigenVector(q_all, q);
        modHelp::yarpToEigenVector(dq_all,dq);
        modHelp::yarpToEigenVector(ddq_all,ddq);

        //set q_all inside icub model
        model->setJointPositions(q); // set joints values on icub model fixed
        model->setJointVelocities(dq); // set also velocities

        // now send to icubsim
        // init icubsim left hand with the initial point of the model
        //Vector xLA_des;
        //xLA_des.resize(model.nbInternalDofs(),0.0);
        //modHelp::eigenToYarpVector(q, xLA_des);
        //icrtLA->goToPosition(xLA_des);


//        Eigen::Vector3d e0 = SF->getPosition().getTranslation() - TF->getPosition().getTranslation();
//        cout<<"err lhand2:"<< e0.transpose()<<endl;
        cout<<"pos SF:"<< SF->getPosition().getTranslation().transpose()<<endl;
//        cout<<"pos TF:"<< TF->getPosition().getTranslation().transpose()<<endl;
        cout<<"err lhand:"<< accTask2->getError().transpose()<<endl;
        //cout<<"pos rhand: "<< model->getSegmentPosition(16).getTranslation().transpose()<<"\n";
//        cout<<"pos lhand: "<< model->getSegmentPosition(23).getTranslation().transpose()<<"\n";


        ctrl.computeOutput(tau);    //compute tau
//        cout<<"tau: "<<tau.transpose()<<"\n";

         // FINALLY SEND COMMANDS TO ICUBSIM
         Vector sentTau(7,0.0);
         std::cout << "Tau ";
         for(unsigned int i=0; i <7; i++){
             sentTau(i) = tau(i+16);
             std::cout << sentTau(i) << " ";
         }
         std::cout << std::endl;
         //itorqueLA->setRefTorques(sentTau.data());
         Vector tauLA(7,0.0);
         Vector tauRA(7,0.0);
         Vector tauHE(3,0.0);
         Vector tauLL(6,0.0);
         Vector tauRL(6,0.0);
         Vector tauTO(3,0.0);

         modHelp::eigenToYarpVector(tau.segment(16,7), tauLA);
         modHelp::eigenToYarpVector(tau.segment(9,7), tauRA);
         modHelp::eigenToYarpVector(tau.segment(23,3), tauHE);
         modHelp::eigenToYarpVector(tau.segment(0,6), tauLL);
         modHelp::eigenToYarpVector(tau.segment(26,6), tauRL);
         modHelp::eigenToYarpVector(tau.segment(6,3), tauTO);

         itorqueLA->setRefTorques(tauLA.data());
         itorqueRA->setRefTorques(tauRA.data());
         itorqueHE->setRefTorques(tauHE.data());
         itorqueLL->setRefTorques(tauLL.data());
         itorqueRL->setRefTorques(tauRL.data());
         itorqueTO->setRefTorques(tauTO.data());

    }

    // end of simulation
    //q = model.getJointPositions();
    //Get position and orientation of the right arm
    //Vector xLA,oLA;
    //xLA.resize(3, 0.0);
    //oLA.resize(4, 0.0);
    //icrtLA->getPose(xLA,oLA);

    //cout<<"FINAL RESULT"<<endl
    //    <<"model   = "<<q<<endl
    //    <<"icubsim = "<<xLA.toString()<<endl;


    //---------------------------------------------------------------------------------
    // closing iCubSim and YARP stuff

    cout<<"Closing drivers"<<endl;

    //if(using_cartesian_arm_left)
    //{
    //   icrtLA->stopControl(); 
    //   icrtLA->restoreContext(startup_context_id_RA);
    //   deleteDriver(ddCartLA);
    //}
    //deleteDriver(ddRA);

    cout<<"ISIRControllerYARPTest is over"<<endl;

    return 0;
}
