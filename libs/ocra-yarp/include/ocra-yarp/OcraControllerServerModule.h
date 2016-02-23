/*! \file       OcraControllerServerModule.h
 *  \brief      Module class for the controller server.
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

#ifndef OCRA_CONTROLLER_SERVER_THREAD
#define OCRA_CONTROLLER_SERVER_THREAD

#include <iostream>

#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>


#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "ocra-yarp/OcraControllerServerThread.h"

using namespace yarp::os;
using namespace wbi;

namespace ocra_yarp
{

class OcraControllerServerModule: public RFModule
{
    /* module parameters */
    std::string moduleName;
    std::string robotName;
    std::string startupTaskSetPath;
    std::string startupSequence;
    bool debugMode, isFloatingBase;
    int period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;

    OcraControllerServerThread* ctrlThread; // locomotion control thread
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    OcraControllerServerModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();
    bool updateModule();
    void printHelp();

};

} // namespace ocra_yarp

#endif //OCRA_CONTROLLER_SERVER_THREAD
