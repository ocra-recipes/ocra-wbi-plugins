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

#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


#include "orcisir/Solvers/OneLevelSolver.h"
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
    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);
    fb_torque.resize(robot->getDoFs());
    printPeriod = options.check("printPeriod",Value(1000.0),"Print a debug message every printPeriod milliseconds.").asDouble();

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    
    //================ SET UP CONTROLLER ========================================================================//

    bool useReducedProblem = false;
	orcisir::OneLevelSolverWithQuadProg   internalSolver;
    ctrl = new orcisir::ISIRController("icubControl", *orcModel, internalSolver, useReducedProblem);

	//================ FULL STATE ==============================================================================//
    orc::FullModelState*   FMS;
    FMS = new orc::FullModelState("torqueTask.FModelState", *orcModel, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState*  FTS;
    FTS = new orc::FullTargetState("torqueTask.FTargetState", *orcModel, orc::FullState::INTERNAL);
    orc::FullStateFeature* feat;
    feat = new orc::FullStateFeature("torqueTask", *FMS);
    orc::FullStateFeature* featDes;
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);
    
    FTS->set_q(Eigen::VectorXd::Constant(orcModel->nbInternalDofs(), 0.0));

    orcisir::ISIRTask* accTask;
    accTask = &(ctrl->createISIRTask("accTask", *feat, *featDes));
    accTask->initAsAccelerationTask();
    ctrl->addTask(*accTask);
    accTask->activateAsObjective();
    accTask->setStiffness(100);
    accTask->setDamping(4);
    accTask->setWeight(0.01);

	//================ CoM Task ===========================================================================//
	

	//Create a CoM frame	
    orc::CoMFrame*        CoM_F;
    CoM_F = new orc::CoMFrame("frame.CoM_Frame", *orcModel);
    
	//Create a Target frame for the CoM (here it coincides with the CoM)	
    orc::TargetFrame*         CoM_TF;
    CoM_TF = new orc::TargetFrame("frame.CoM_TFrame", *orcModel);
    
	//Create a positioning control feature for the frame	
    orc::PositionFeature* CoM_feat;
    CoM_feat = new orc::PositionFeature("CoM_frame", *CoM_F, orc::XYZ);
    
	//Create a desired position for the task <-generates the error term and so the accelerations
    orc::PositionFeature* CoM_featDes;
    CoM_featDes = new orc::PositionFeature("CoM_frame.Des", *CoM_TF, orc::XYZ);
    
    //Set target frame variables
    CoM_TF->setPosition(Eigen::Displacementd(0.0,0.0,0.6));
    CoM_TF->setVelocity(Eigen::Twistd());
    CoM_TF->setAcceleration(Eigen::Twistd());

	//Create the controler's task
    orcisir::ISIRTask* CoM_accTask;
    CoM_accTask = &(ctrl->createISIRTask("CoM_accTask", *CoM_feat, *CoM_featDes));

    CoM_accTask->initAsAccelerationTask();
    ctrl->addTask(*CoM_accTask);
    
    CoM_accTask->activateAsObjective();
    CoM_accTask->setStiffness(200);//2000000000
    CoM_accTask->setDamping(80);
    CoM_accTask->setWeight(100.0);
    

	//================ Cartesian Frame Task ================================================================//
/*	
    orc::SegmentFrame*        SF;
    std::cout<<"Test 1 \n";
    SF = new orc::SegmentFrame("frame.SFrame", *orcModel, "lap_belt_1", Eigen::Displacementd());
    std::cout<<"Test 2 \n";
    orc::TargetFrame*         TF;
    TF = new orc::TargetFrame("frame.TFrame", *orcModel);
    std::cout<<"Test 3 \n";
    orc::PositionFeature* feat2;
    //feat2 = new orc::PositionFeature("frame", *SF, orc::XYZ);
    std::cout<<"Test 4 \n";
    orc::PositionFeature* featDes2;
    featDes2 = new orc::PositionFeature("frame.Des", *TF, orc::XYZ);
	std::cout<<"Test 5 \n";
    TF->setPosition(Eigen::Displacementd(0.0,0.0,0.6));
    TF->setVelocity(Eigen::Twistd());
    TF->setAcceleration(Eigen::Twistd());
	std::cout<<"Test 6 \n";
    orcisir::ISIRTask* accTask2;
    std::cout<<"Test 7 \n";
    accTask2 = &(ctrl->createISIRTask("accTask2", *feat2, *featDes2));
    std::cout<<"Test 8 \n";
    accTask2->initAsAccelerationTask();
    ctrl->addTask(*accTask2);
    accTask2->activateAsObjective();
    accTask2->setStiffness(200);//2000000000
    accTask2->setDamping(80);
    accTask2->setWeight(100.0);
	std::cout<<"Test 4 \n";
*/	
	//==================================================================================================//

	return true;
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::run()
{
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
	std::cout<<"Test 1 \n";
	ctrl->computeOutput(eigenTorques);
	std::cout<<"Test 2 \n";
	modHelp::eigenToYarpVector(eigenTorques, torques_cmd);


    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

    if(printCountdown==0) {
        std::cout << "The robot encoders values are: " << std::endl;
        std::cout << fb_qRad.transpose() << std::endl;
        std::cout << "The joint torquess are" << std::endl;
        std::cout << fb_torque.toString() << std::endl;
        

        std::cout << "Data in orcModel" << std::endl;
        //orcModel->printAllData();


    }
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::threadRelease()
{
}
