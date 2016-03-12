/*! \file       main.cpp
 *  \brief      Basically just launches the server module which then launches the server thread. (...it's a dream within a dream)
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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>

#include <ocra-icub-server/Module.h>

#define DEFAULT_YARP_CONTEXT "ocra-icub-server"



int main (int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    Module module;
    yarp::os::Network yarp;
    yarp::os::Log yLog;


    rf.setVerbose(true);
    rf.setDefaultConfigFile("ocra-icub-server.ini"); //default config file name.
    rf.setDefaultContext(DEFAULT_YARP_CONTEXT); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {

        module.printHelp();

        return 0;
    }


    double network_timeout = 10.0;
    if (!yarp.checkNetwork(network_timeout))
    {
        yLog.fatal() << "YARP network is not available";
        return -1;
    }


    return module.runModule(rf);
}
