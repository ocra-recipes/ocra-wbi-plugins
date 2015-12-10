/*
* Copyright (C) 2013 ISIR
* Author: Darwin Lau, MingXing Liu, Ryan Lober
* email: lau@isir.upmc.fr, liu@isir.upmc.fr, lober@isir.upmc.fr
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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ISIRObservers/module.h>

#define DEFAULT_YARP_CONTEXT "observerModule"

using namespace yarp::os;
using namespace std;

int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("wocraController.ini"); //default config file name.
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
        cout<< "\t--floatingBase :If this flag is present then the controller will run in using a floating base dynamic model and control. Defaults to false, or fixed base if no flag is present." <<endl;

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
    observerModule module;

    return module.runModule(rf);
}
