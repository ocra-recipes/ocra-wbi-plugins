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
    std::cout<< "Test 1 \n";
    bool useReducedProblem = false;
	orcisir::OneLevelSolverWithQuadProg   internalSolver;
	std::cout<< "Test 2 \n";
    ctrl = new orcisir::ISIRController("icubControl", *orcModel, internalSolver, useReducedProblem);
	std::cout<< "Test 3 \n";
	//================ FULL STATE ==============================================================================//
    orc::FullModelState*   FMS;
    FMS = new orc::FullModelState("torqueTask.FModelState", *orcModel, orc::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    orc::FullTargetState*  FTS;
    FTS = new orc::FullTargetState("torqueTask.FTargetState", *orcModel, orc::FullState::INTERNAL);
    orc::FullStateFeature* feat;
    feat = new orc::FullStateFeature("torqueTask", *FMS);
    orc::FullStateFeature* featDes;
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);
	std::cout<< "Test 4 \n";
    FTS->set_q(Eigen::VectorXd::Constant(orcModel->nbInternalDofs(), 0.0));
	std::cout<< "Test 5 \n";

    orcisir::ISIRTask* accTask;
    std::cout<< "Test 6 \n";
    accTask = &(ctrl->createISIRTask("accTask", *feat, *featDes));
    accTask->initAsAccelerationTask();
    ctrl->addTask(*accTask);
    accTask->activateAsObjective();
    accTask->setStiffness(100);
    accTask->setDamping(4);
    accTask->setWeight(0.01);
    std::cout<< "Test 7 \n";
/*
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
*/

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
    //std::cout<<"eigenTorques.size() == torques_cmd().size() " << eigenTorques.rows() << robot->getDoFs();
	//ctrl->computeOutput(eigenTorques);
	//modHelp::eigenToYarpVector(eigenTorques, torques_cmd);


    // setControlReference(double *ref, int joint) to set joint torque (in torque mode)
    robot->setControlReference(torques_cmd.data());

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print

    if(printCountdown==0) {
        std::cout << "The robot encoders values are: " << std::endl;
        std::cout << fb_qRad.transpose() << std::endl;
        std::cout << "The joint torquess are" << std::endl;
        std::cout << fb_torque.toString() << std::endl;
        

        std::cout << "Data in orcModel" << std::endl;
        orcModel->printAllData();


    }
}

//*************************************************************************************************************************
void basicWholeBodyControlThread::threadRelease()
{
}
