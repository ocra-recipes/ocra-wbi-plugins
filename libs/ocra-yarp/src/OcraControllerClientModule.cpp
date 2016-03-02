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

        }
    }

    // Connect to controller.

    // If running get controller data and wbi_config_file path

    // construct wbi options

    // pass to client thread.
}

OcraControllerClientModule::~OcraControllerClientModule()
{
}

bool OcraControllerClientModule::configure(yarp::os::ResourceFinder &rf)
{

    return true;
}


bool OcraControllerClientModule::interruptModule()
{

    return true;
}

bool OcraControllerClientModule::close()
{

    return true;
}

bool OcraControllerClientModule::updateModule()
{

    return true;
}

void OcraControllerClientModule::printHelp()
{

}
