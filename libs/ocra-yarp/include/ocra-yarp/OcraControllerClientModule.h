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

#ifndef OCRA_CONTROLLER_CLIENT_MODULE_H
#define OCRA_CONTROLLER_CLIENT_MODULE_H

#include <iostream>
#include <memory>

#include <yarp/os/RFModule.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "ocra-yarp/OcraControllerClientThread.h"
#include "ocra-yarp/OcraYarpTools.h"


namespace ocra_yarp
{
/*! \class OcraControllerClientModule
 *  \brief The controller module which launches the controller thread.
 *
 *  Basically all this does is parse the command line arguments and look for the various config and task set files. It then instantiates a WBI instance (yarpWBI specifically) and a \ref OcraControllerClientThread instance. It launches these threads and then basically just waits till it gets a kill (ctrl+c) command to close them down. Does a little keeping track of time as well.
 */
class OcraControllerClientModule: public yarp::os::RFModule
{
DEFINE_CLASS_POINTER_TYPEDEFS(OcraControllerClientModule)

public:
    /*! Constructor which essentially does nothing.
     */
    OcraControllerClientModule(OcraControllerClientThread::shared_ptr customClientThread);

    /*! Destructor which essentially does nothing.
     */
    ~OcraControllerClientModule();

    /*! Configures the module by parsing the RF contents.
     *  \param rf A resource finder instance which is initialized from the command line args.
     *
     *  \return True or false if the configuration was successful.
     */
    bool configure(yarp::os::ResourceFinder &rf);

    /*! Interrupts the module execution and stops the control and wbi threads.
     */
    bool interruptModule();

    /*! Closes the module. First shuts down the threads.
     */
    bool close();

    /*! Updates the OcraControllerClientModule. Basically just clocks the thread run() method.
     *  \return Whether or not the clocking functions worked.
     */
    bool updateModule();

    /*! Prints all the command line args one could use.
     */
    void printHelp();

private:
    OcraControllerClientThread::shared_ptr clientThread;
};

} // namespace ocra_yarp

#endif //OCRA_CONTROLLER_CLIENT_MODULE_H
