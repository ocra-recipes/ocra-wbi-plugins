/*! \file       Thread.cpp
 *  \brief      The thread class for the controller server.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-yarp.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ocra-icub-server/Thread.h"





OcraControllerOptions::OcraControllerOptions()
: threadPeriod(10)
, serverName("")
, robotName("")
, startupTaskSetPath("")
, startupSequence("")
, wbiConfigFilePath("")
, runInDebugMode(false)
, isFloatingBase(false)
, yarpWbiOptions(yarp::os::Property())
, controllerType(ocra_recipes::WOCRA_CONTROLLER)
, solver(ocra_recipes::QUADPROG)
{
}

OcraControllerOptions::~OcraControllerOptions()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Constructor
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
Thread::Thread(OcraControllerOptions& controller_options, std::shared_ptr<wbi::wholeBodyInterface> wbi)
: RateThread(controller_options.threadPeriod)
, ctrlOptions(controller_options)
, controllerStatus(ocra_icub::CONTROLLER_SERVER_STOPPED)
{
    yarpWbi = wbi;
    bool usingInterprocessCommunication = true;
    ctrlServer = std::make_shared<IcubControllerServer>(  yarpWbi,
                                        ctrlOptions.robotName,
                                        ctrlOptions.isFloatingBase,
                                        ctrlOptions.controllerType,
                                        ctrlOptions.solver,
                                        usingInterprocessCommunication);

    ctrlServer->initialize();


    // Construct rpc server callback and bind to the control thread.
    rpcServerCallback = std::make_shared<ControllerRpcServerCallback>(*this);
    // Open the rpc server port.
    rpcServerPort.open("/IcubControllerServer/info/rpc:i");
    // Bind the callback to the port.
    rpcServerPort.setReader(*rpcServerCallback);
}

Thread::~Thread()
{
    rpcServerPort.close();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Init
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thread::threadInit()
{
    ctrlServer->addTaskManagersFromXmlFile(ctrlOptions.startupTaskSetPath);
    minTorques      = Eigen::ArrayXd::Constant(yarpWbi->getDoFs(), TORQUE_MIN);
    maxTorques      = Eigen::ArrayXd::Constant(yarpWbi->getDoFs(), TORQUE_MAX);
    initialPosture  = Eigen::VectorXd::Zero(yarpWbi->getDoFs());
    yarpWbi->getEstimates(wbi::ESTIMATE_JOINT_POS, initialPosture.data(), ALL_JOINTS);

	return yarpWbi->setControlMode(wbi::CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    controllerStatus = ocra_icub::CONTROLLER_SERVER_RUNNING;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Run
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Thread::run()
{
	ctrlServer->computeTorques(torques);
    torques = ((torques.array().max(minTorques)).min(maxTorques)).matrix().eval();
    yarpWbi->setControlReference(torques.data());
}

void Thread::threadRelease()
{
    controllerStatus = ocra_icub::CONTROLLER_SERVER_STOPPED;

    yarpWbi->setControlMode(wbi::CTRL_MODE_POS, initialPosture.data(), ALL_JOINTS);
    yarpWbi->setControlReference(initialPosture.data());

}




void Thread::parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        // OCRA_CONTROLLER_MESSAGE message();
        switch (input.get(i).asInt()) {
            case ocra_icub::GET_CONTROLLER_SERVER_STATUS:
                {
                    std::cout << "Got message: GET_CONTROLLER_SERVER_STATUS." << std::endl;
                    reply.addInt(controllerStatus);
                    ++i;
                }break;

            case ocra_icub::GET_MODEL_CONFIG_INFO:
                {
                    std::cout << "Got message: GET_MODEL_CONFIG_INFO." << std::endl;
                    reply.addString(ctrlOptions.wbiConfigFilePath);
                    reply.addString(ctrlOptions.robotName);
                    reply.addInt(ctrlOptions.isFloatingBase);
                    ++i;
                }break;

            case ocra_icub::HELP:
                {
                    ++i;
                    std::cout << "Got message: HELP." << std::endl;
                }break;

            default:
                {
                    ++i;
                    std::cout << "Got message: UNKNOWN." << std::endl;
                }break;

        }
    }
}

/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
Thread::ControllerRpcServerCallback::ControllerRpcServerCallback(Thread& threadRef)
: thread(threadRef)
{
    //do nothing
}

bool Thread::ControllerRpcServerCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        thread.parseIncomingMessage(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/
