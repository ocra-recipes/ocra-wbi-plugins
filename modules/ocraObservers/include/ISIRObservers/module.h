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

#ifndef OBSERVERMODULE_H
#define OBSERVERMODULE_H

#include <iostream>

#include <yarp/os/RFModule.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "thread.h"

using namespace yarp::os;
using namespace wbi;


class observerModule: public RFModule
{
    /* module parameters */
    std::string moduleName;
    std::string robotName;

    int period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;

    observerThread* ctrlThread; // locomotion control thread
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    observerModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();
    bool updateModule();

};


#endif
//empty line to make gcc happy
