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

#include "ocra-yarp/OcraControllerServerModule.h"

// YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarpWbi;
using namespace ocra_yarp;

OcraControllerServerModule::OcraControllerServerModule()
{
    ctrlThread = 0;
    robotInterface = 0;
    period = 10;
}

bool OcraControllerServerModule::configure(ResourceFinder &rf)
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
        std::string xmlExt = ".xml";
        std::string fileName = rf.find("taskSet").asString().c_str();

        std::size_t fileExtensionStart = fileName.find_last_of(".");
        if (fileExtensionStart == std::string::npos) {
            fileName += xmlExt;
        }else{
            std::string extension = fileName.substr(fileExtensionStart);
            if (extension != xmlExt) {
                fileName = fileName.substr(0, fileExtensionStart) + xmlExt;
            }
        }
        std::size_t subDirEnd = fileName.find_first_of("/");
        if (subDirEnd == std::string::npos) {
            fileName = "taskSets/" + fileName;
        }else{
            std::string subDir = fileName.substr(0,subDirEnd);
            if (subDir != "taskSets/") {
                fileName = "taskSets/" + fileName.substr(subDirEnd+1);
            }
        }

        startupTaskSetPath = rf.findFileByName(fileName).c_str();
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

    ctrlThread = new OcraControllerServerThread(moduleName,
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


bool OcraControllerServerModule::interruptModule()
{
    if(ctrlThread)
        ctrlThread->suspend();
    return true;
}

bool OcraControllerServerModule::close()
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

bool OcraControllerServerModule::updateModule()
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
        // printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
        // printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }
//#endif

    return true;
}

double OcraControllerServerModule::getPeriod()
{
    return period;
}

void OcraControllerServerModule::printHelp()
{
    std::cout<< "Possible parameters" << std::endl << std::endl;
    std::cout<< "\t--context :Where to find an user defined .ini file e.g. /locomotionCtrl" <<std::endl;
    std::cout<< "\t--from :Name of the file.ini to be used for configuration." <<std::endl;
    std::cout<< "\t--rate :Period used by the module. Default set to 10ms." <<std::endl;
    std::cout<< "\t--robot :Robot name (icubSim or icub). Set to icub by default." <<std::endl;
    std::cout<< "\t--local :Prefix of the ports opened by the module. Set to the module name by default, i.e. basicWholeBodyInterfaceModule." <<std::endl;
    std::cout<< "\t--taskSet :A path to an XML file containing a set of tasks. The tasks will be created when the controller is started. Set to empty by default." <<std::endl;
    std::cout<< "\t--sequence :A string identifying a predefined scenario. The scenarios (sets of tasks and control logic) are defined in sequenceCollection and will be created when the controller is started. Set to empty by default." <<std::endl;
    std::cout<< "\t--debug :If this flag is present then the controller will run in Debug mode which allows each joint to be tested individually." <<std::endl;
    std::cout<< "\t--floatingBase :If this flag is present then the controller will run in using a floating base dynamic model and control. Defaults to false, or fixed base if no flag is present." <<std::endl;
}
