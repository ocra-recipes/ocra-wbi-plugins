/*! \file       OcraControllerClientModule.h
 *  \brief      Module class for the controller Client.
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

#include "ocra-yarp/OcraControllerClientModule.h"

using namespace ocra_yarp;


OcraControllerClientModule::OcraControllerClientModule(OcraControllerClientThread::shared_ptr customClientThread)
{
    clientThread = customClientThread;
    expectedClientThreadPeriod = clientThread->getExpectedPeriod();

    // open controller connection
    ControllerConnection ctrlCon;
    bool openTaskPorts = false;
    if (ctrlCon.open(openTaskPorts))
    {
        yarp::os::Bottle reply;
        reply = ctrlCon.queryController(OCRA_CONTROLLER_MESSAGE::GET_CONTROLLER_STATUS);
        if (reply.get(0).asInt() == OCRA_CONTROLLER_MESSAGE::CONTROLLER_RUNNING) {
            reply = ctrlCon.queryController(OCRA_CONTROLLER_MESSAGE::GET_WBI_CONFIG_FILE_PATH);
            std::string wbiConfigFilePath = reply.get(0).asString();

            reply = ctrlCon.queryController(OCRA_CONTROLLER_MESSAGE::GET_ROBOT_NAME);
            std::string robotName = reply.get(0).asString();

            std::cout << "Found WBI config file here: " << wbiConfigFilePath << std::endl;
            yarp::os::Property yarpWbiOptions;

            yarpWbiOptions.fromConfigFile(wbiConfigFilePath);
            // Overwrite the robot parameter that could be present in wbi_conf_file
            yarpWbiOptions.put("robot", robotName);

            clientThread->setWbiOptions(yarpWbiOptions);

            // Construct rpc server callback and bind to the control thread.
            rpcCallback = std::make_shared<moduleCallback>(*this);
            // Open the rpc server port.
            rpcPort.open("/OCRA/ClientModule/rpc:i");
            // Bind the callback to the port.
            rpcPort.setReader(*rpcCallback);

        }
    }
}

OcraControllerClientModule::~OcraControllerClientModule()
{
    rpcPort.close();
}

bool OcraControllerClientModule::configure(yarp::os::ResourceFinder &rf)
{

    return true;
}


bool OcraControllerClientModule::interruptModule()
{
    if(clientThread)
        clientThread->suspend();
    return true;
}

bool OcraControllerClientModule::close()
{
    /* Stop the control thread. */
    if(clientThread){
        clientThread->stop();
    }

    /* Print performance information */
    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", expectedClientThreadPeriod, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTime<0.5*(double)expectedClientThreadPeriod)
        printf("Next time you could set a lower period to improve the controller performance.\n");
    else if(avgTime>1.3*(double)expectedClientThreadPeriod)
        printf("The period you set was impossible to attain. Next time you could set a higher period.\n");
    return true;
}

bool OcraControllerClientModule::updateModule()
{
    // Get the average time between two calls of the RateThread.run() method.
    clientThread->getEstPeriod(avgTime, stdDev);

    // Get the average time the .run() method takes to compute the control.
    clientThread->getEstUsed(avgTimeUsed, stdDevUsed);

    // If the period of the control thread is too slow then print a warning.
    if(avgTime > 1.3*(double)expectedClientThreadPeriod)
    {
        yLog.warning() << "CLIENT THREAD LOOP IS TOO SLOW\nReal period: "<< avgTime <<"+/-"<< stdDev <<"\nExpected period: " << expectedClientThreadPeriod <<"\nDuration of 'run' method: "<<avgTimeUsed<<"+/-"<< stdDevUsed<<"\n";
    }

    return customUpdateModule();
}

void OcraControllerClientModule::printHelp()
{

}

void OcraControllerClientModule::callbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply)
{
    if (message.size() != 0) {
        reply.clear();
        customCallbackParser(message, reply);
    }
}

void OcraControllerClientModule::customCallbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply)
{
    // Do nothing if not implemented.
}
bool OcraControllerClientModule::customUpdateModule()
{
    // Do nothing if not implemented.
    return true;
}


/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
OcraControllerClientModule::moduleCallback::moduleCallback(OcraControllerClientModule& newModuleRef)
: moduleRef(newModuleRef)
{
    //do nothing
}

bool OcraControllerClientModule::moduleCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        moduleRef.callbackParser(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/