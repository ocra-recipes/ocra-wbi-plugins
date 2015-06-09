/*
* Copyright (C) 2014 ...
* Author: ...
* email: ...
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

#ifndef GOCRA_CONTROLLER_MODULE_H
#define GOCRA_CONTROLLER_MODULE_H

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>


#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "thread.h"

using namespace std;
using namespace yarp::os;
using namespace wbi;

namespace gOcraController
{

class gOcraControllerModule: public RFModule
{
    /* module parameters */
    string moduleName;
    string robotName;
    int period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;

    gOcraControllerThread* ctrlThread; // control thread
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    gOcraControllerModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();
    bool updateModule();

};

}

#endif
//empty line to make gcc happy
