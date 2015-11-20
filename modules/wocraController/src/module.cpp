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

#include <wocraController/thread.h>
#include <wocraController/module.h>

// YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarpWbi;
using namespace wocraController;

wocraControllerModule::wocraControllerModule()
{
    ctrlThread = 0;
    robotInterface = 0;
    period = 10;
}

bool wocraControllerModule::configure(ResourceFinder &rf)
{
    //--------------------------READ FROM CONFIGURATION----------------------
    if( rf.check("robot") )
    {
        robotName = rf.find("robot").asString().c_str();
        std::cout << "\n\nRobot name is: " << robotName << "\n" << std::endl;
    }
    if( rf.check("local") )
    {
        moduleName = rf.find("local").asString().c_str();
    }

    if( rf.check("taskSet") )
    {
        startupTaskSetPath = rf.findFile("taskSet").c_str();
    }

    if( rf.check("sequence") )
    {
        startupSequence = rf.find("sequence").asString().c_str();
    }

    if( rf.check("debug") )
    {
        debugMode = true;
    }else{debugMode = false;}

    if( rf.check("floatingBase") )
    {
        isFloatingBase = true;
    }else{isFloatingBase = false;}

    yarp::os::Property yarpWbiOptions;
    // Get wbi options from the canonical file
    if ( !rf.check("wbi_conf_file") )
    {
        fprintf(stderr, "[ERR] wocraController: Impossible to open wholeBodyInterface: wbi_conf_file option missing");
    }
    std::string wbiConfFile = rf.findFile("wbi_conf_file");
    yarpWbiOptions.fromConfigFile(wbiConfFile);
    // Overwrite the robot parameter that could be present in wbi_conf_file
    yarpWbiOptions.put("robot", robotName);
    robotInterface = new yarpWholeBodyInterface(moduleName.c_str(), yarpWbiOptions);

    IDList robotJoints;
    std::string robotJointsListName = "ROBOT_MAIN_JOINTS";
    if(!loadIdListFromConfig(robotJointsListName, yarpWbiOptions, robotJoints))
    {
        fprintf(stderr, "[ERR] wocraController: Impossible to load wbiId joint list with name %s\n", robotJointsListName.c_str());
    }
    robotInterface->addJoints(robotJoints);


    // Make sure all the add* functions are done before the "init"
    if(!robotInterface->init())
    {
        fprintf(stderr, "Error while initializing whole body interface. Closing module\n"); return false;
    }

    //--------------------------CTRL THREAD--------------------------
    yarp::os::Property controller_options;
    //If the printPeriod is found in the options, send it to the controller
    if( rf.check("printPeriod") && rf.find("printPeriod").isDouble() )
    {
        controller_options.put("printPeriod",rf.find("printPeriod").asDouble());
    }

    ctrlThread = new wocraControllerThread(moduleName,
                                                   robotName,
                                                   period,
                                                   robotInterface,
                                                   controller_options,
                                                   startupTaskSetPath,
                                                   startupSequence,
                                                   debugMode,
                                                   isFloatingBase);
    if(!ctrlThread->start())
    { 
        fprintf(stderr, "Error while initializing wocraController thread. Closing module.\n"); return false;
    }

    fprintf(stderr,"wocraController thread started\n");

    return true;
}


bool wocraControllerModule::interruptModule()
{
    if(ctrlThread)
        ctrlThread->suspend();
    return true;
}

bool wocraControllerModule::close()
{
//stop threads
    if(ctrlThread){ ctrlThread->stop(); delete ctrlThread; ctrlThread = 0; }

    if(robotInterface)
    {
        bool res=robotInterface->close();
        if(!res)
            printf("Error while closing robot interface\n");
        delete robotInterface;
        robotInterface = 0;
    }
    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*period)
        printf("Next time you could set a lower period to improve the controller performance.\n");
    else if(avgTime>1.3*period)
        printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

    return true;
}

bool wocraControllerModule::updateModule()
{
    if (ctrlThread==0)
    {
        printf("ControlThread pointers are zero\n");
        return false;
    }

    ctrlThread->getEstPeriod(avgTime, stdDev);
    ctrlThread->getEstUsed(avgTimeUsed, stdDevUsed); // real duration of run()
//#ifndef NDEBUG
    if(avgTime > 1.3 * period)
    {
        printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
        printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }
//#endif

    return true;
}

double wocraControllerModule::getPeriod()
{
    return period;
}
