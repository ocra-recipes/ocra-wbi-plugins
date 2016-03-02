/*! \file       OcraControllerClientThread.cpp
 *  \brief      A base class for all controller clients.
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

#include "ocra-yarp/OcraControllerClientThread.h"


using namespace ocra_yarp;

int OcraControllerClientThread::CONTROLLER_CLIENT_THREAD_COUNT = 0;

OcraControllerClientThread::OcraControllerClientThread(const int period)
: RateThread(period)
, expectedPeriod(period)
{

}

OcraControllerClientThread::~OcraControllerClientThread()
{

}

bool OcraControllerClientThread::threadInit()
{
    return client_threadInit();
}
void OcraControllerClientThread::threadRelease()
{
    client_threadRelease();
}
void OcraControllerClientThread::run()
{
    client_run();
}

void OcraControllerClientThread::setWbiOptions(yarp::os::Property& wbiOptions)
{
    yarpWbiOptions = wbiOptions;

    // Create the wholeBodyInterface.
    std::string robotInterfaceName = "ControllerClientThreadWBI_"+std::to_string(OcraControllerClientThread::CONTROLLER_CLIENT_THREAD_COUNT);
    robotInterface = std::make_shared<yarpWbi::yarpWholeBodyInterface>(robotInterfaceName.c_str(), yarpWbiOptions);

    // Add the robot's specific joints to the WBI.
    wbi::IDList robotJoints;
    std::string robotJointsListName = "ROBOT_MAIN_JOINTS";
    if(!yarpWbi::loadIdListFromConfig(robotJointsListName, yarpWbiOptions, robotJoints))
    {
        yLog.error() << "Impossible to load wbiId joint list with name: " << robotJointsListName;
    }
    robotInterface->addJoints(robotJoints);
}
