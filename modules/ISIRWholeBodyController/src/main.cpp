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
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ISIRWholeBodyController/module.h>

#define DEFAULT_YARP_CONTEXT "ISIRWholeBodyController"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace ISIRWholeBodyController;


int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("ISIRWholeBodyController.ini"); //default config file name.
    rf.setDefaultContext(DEFAULT_YARP_CONTEXT); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<< "Possible parameters" << endl << endl;
        cout<< "\t--context :Where to find an user defined .ini file e.g. /locomotionCtrl" <<endl;
        cout<< "\t--from :Name of the file.ini to be used for configuration." <<endl;
        cout<< "\t--rate :Period used by the module. Default set to 10ms." <<endl;
        cout<< "\t--robot :Robot name (icubSim or icub). Set to icub by default." <<endl;
        cout<< "\t--local :Prefix of the ports opened by the module. Set to the module name by default, i.e. basicWholeBodyInterfaceModule." <<endl;
        cout<< "\t--taskSet :A path to an XML file containing a set of tasks. The tasks will be created when the controller is started. Set to empty by default." <<endl;
        cout<< "\t--sequence :A string identifying a predefined scenario. The scenarios (sets of tasks and control logic) are defined in sequenceCollection and will be created when the controller is started. Set to empty by default." <<endl;
        cout<< "\t--debug :If this flag is present then the controller will run in Debug mode which allows each joint to be tested individually." <<endl;
        return 0;
    }

    Network yarp;

    double network_timeout = 10.0;
    if (!yarp.checkNetwork(network_timeout))
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    ISIRWholeBodyControllerModule module;

    return module.runModule(rf);
}
