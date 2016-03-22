/*! \file       OcraControllerClientThread.cpp
 *  \brief      A base class for all controller clients.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-icub.
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

#include "ocra-icub/OcraControllerClientThread.h"


using namespace ocra_yarp;

int OcraControllerClientThread::CONTROLLER_CLIENT_THREAD_COUNT = 0;

OcraControllerClientThread::OcraControllerClientThread(const int period)
: RateThread(period)
, expectedPeriod(period)
{
    // Increment the thread counter and save it in thread number.
    threadNumber = ++OcraControllerClientThread::CONTROLLER_CLIENT_THREAD_COUNT;

    // Construct rpc server callback and bind to the control thread.
    rpcCallback = std::make_shared<threadCallback>(*this);
    // Make the port name
    rpcPortName = "/OCRA/" + getThreadName() + "/rpc:i";
    // Open the rpc server port.
    rpcPort.open(rpcPortName);
    // Bind the callback to the port.
    rpcPort.setReader(*rpcCallback);
}

OcraControllerClientThread::~OcraControllerClientThread()
{
    rpcPort.close();
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

int OcraControllerClientThread::getExpectedPeriod()
{
    return expectedPeriod;
}

bool OcraControllerClientThread::startModelThread()
{
}

void OcraControllerClientThread::stopModelThread()
{
}


std::string OcraControllerClientThread::getThreadName()
{
    return "ControllerClientThread_"+std::to_string(threadNumber);
}

void OcraControllerClientThread::setWbiOptions(yarp::os::Property& wbiOptions, const bool floatingBase)
{
    yarpWbiOptions = wbiOptions;
    isFloatingBase = floatingBase;

    // Create the wholeBodyInterface.
    std::string robotInterfaceName = getThreadName() + "_WBI/";
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

void OcraControllerClientThread::callbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply)
{
    if (message.size() != 0) {
        reply.clear();
        customCallbackParser(message, reply);
    }
}

void OcraControllerClientThread::customCallbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply)
{
    // Do nothing if not implemented.
}

/**************************************************************************************************
                                    Nested threadCallback Class
**************************************************************************************************/
OcraControllerClientThread::threadCallback::threadCallback(OcraControllerClientThread& newThreadRef)
: threadRef(newThreadRef)
{
    //do nothing
}

bool OcraControllerClientThread::threadCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        threadRef.callbackParser(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/
