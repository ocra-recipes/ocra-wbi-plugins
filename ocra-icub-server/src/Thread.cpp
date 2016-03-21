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
{
    robot = wbi;
    bool usingInterprocessCommunication = true;
    ctrlServer = std::make_shared<IcubControllerServer>(  robot,
                                        ctrlOptions.robotName,
                                        ctrlOptions.isFloatingBase,
                                        ctrlOptions.controllerType,
                                        usingInterprocessCommunication);

    ctrlServer->initialize();
}

Thread::~Thread()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Init
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thread::threadInit()
{
    ctrlServer->addTaskManagersFromXmlFile(ctrlOptions.startupTaskSetPath);
    minTorques      = Eigen::ArrayXd::Constant(robot->getDoFs(), TORQUE_MIN);
    maxTorques      = Eigen::ArrayXd::Constant(robot->getDoFs(), TORQUE_MAX);
    initialPosture  = Eigen::VectorXd::Zero(robot->getDoFs());
    robot->getEstimates(ESTIMATE_JOINT_POS, initialPosture.data(), ALL_JOINTS);

	return robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);
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
    robot->setControlReference(torques.data());
}

void Thread::threadRelease()
{
    robot->setControlMode(CTRL_MODE_POS, initialPosture.data(), ALL_JOINTS);
    robot->setControlReference(initialPosture.data());

}
