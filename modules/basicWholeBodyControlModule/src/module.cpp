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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "thread.h"
#include "module.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace wbiIcub;
using namespace basicWholeBodyControlNamespace;

void iCubVersionFromRf(ResourceFinder & rf, iCub::iDynTree::iCubTree_version_tag & icub_version)
{
    //Checking iCub parts version
    /// \todo this part should be replaced by a more general way of accessing robot parameters
    /// namely urdf for structure parameters and robotInterface xml (or runtime interface) to get available sensors
    icub_version.head_version = 2;
    if( rf.check("headV1") ) {
        icub_version.head_version = 1;
    }
    if( rf.check("headV2") ) {
        icub_version.head_version = 2;
    }

    icub_version.legs_version = 2;
    if( rf.check("legsV1") ) {
        icub_version.legs_version = 1;
    }
    if( rf.check("legsV2") ) {
        icub_version.legs_version = 2;
    }

    /// \note if feet_version are 2, the presence of FT sensors in the feet is assumed
    icub_version.feet_ft = true;
    if( rf.check("feetV1") ) {
        icub_version.feet_ft = false;
    }
    if( rf.check("feetV2") ) {
        icub_version.feet_ft = true;
    }

    if( rf.check("urdf") )
    {
        icub_version.uses_urdf = true;
        icub_version.urdf_file = rf.find("urdf").asString().c_str();
    }
}

basicWholeBodyControlModule::basicWholeBodyControlModule()
{
    ctrlThread = 0;
    robotInterface = 0;
    period = 10;
}

bool basicWholeBodyControlModule::configure(ResourceFinder &rf)
{
    //--------------------------READ FROM CONFIGURATION----------------------
    if( rf.check("robot") )
    {
        robotName = rf.find("robot").asString().c_str();
    }
    if( rf.check("local") )
    {
        moduleName = rf.find("local").asString().c_str();
    }
    //--------------------------WHOLE BODY INTERFACE--------------------------
    iCub::iDynTree::iCubTree_version_tag icub_version;
    iCubVersionFromRf(rf,icub_version);
    robotInterface = new icubWholeBodyInterface(moduleName.c_str(), robotName.c_str(),icub_version);
    robotInterface->addJoints(ICUB_MAIN_JOINTS);
    
    if( rf.check("uses_external_torque_control") )
    {
		if(yarp::os::NetworkBase::exists(string("/jtc/info:o").c_str()))
            printf ("The module jointTorqueControl is running. Proceeding with configuration of the interface...\n");
        
        else{
            printf ("ERROR [mdlStart] >> The jointTorqueControl module is not running... \n");
            return false;
        }

        yarp::os::Value trueValue;
        trueValue.fromString ("true");
        ( (icubWholeBodyInterface*) robotInterface)->setActuactorConfigurationParameter (icubWholeBodyActuators::icubWholeBodyActuatorsUseExternalTorqueModule, trueValue);
        ( (icubWholeBodyInterface*) robotInterface)->setActuactorConfigurationParameter (icubWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleAutoconnect, trueValue);
        ( (icubWholeBodyInterface*) robotInterface)->setActuactorConfigurationParameter (icubWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleName, Value ("jtc"));
    
    
	}
    
    if(!robotInterface->init()){ fprintf(stderr, "Error while initializing whole body interface. Closing module\n"); return false; }

    //--------------------------CTRL THREAD--------------------------
    yarp::os::Property controller_options;

    //If the printPeriod is found in the options, send it to the controller
    if( rf.check("printPeriod") && rf.find("printPeriod").isDouble() )
    {
        controller_options.put("printPeriod",rf.find("printPeriod").asDouble());
    }

    ctrlThread = new basicWholeBodyControlThread(moduleName, robotName, period, robotInterface, controller_options);
    if(!ctrlThread->start()){ fprintf(stderr, "Error while initializing locomotion control thread. Closing module.\n"); return false; }

    fprintf(stderr,"basicWholeBodyControl thread started\n");

    return true;
}


bool basicWholeBodyControlModule::interruptModule()
{
    if(ctrlThread)
        ctrlThread->suspend();
    return true;
}

bool basicWholeBodyControlModule::close()
{
//stop threads
    if(ctrlThread){ ctrlThread->stop(); delete ctrlThread; ctrlThread = 0; }

    if(robotInterface)
    {
        bool res=robotInterface->close();
        if(res)
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

bool basicWholeBodyControlModule::updateModule()
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

double basicWholeBodyControlModule::getPeriod()
{
    return period;
}
