/*! \file       Module.h
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

#ifndef OCRA_CONTROLLER_SERVER_MODULE_H
#define OCRA_CONTROLLER_SERVER_MODULE_H

#include <iostream>
#include <memory>

#include <yarp/os/RFModule.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "ocra-icub-server/Thread.h"
#include "ocra-icub/Utilities.h"


/*! \class Module
 *  \brief The controller module which launches the controller thread.
 *
 *  Basically all this does is parse the command line arguments and look for the various config and task set files. It then instantiates a WBI instance (yarpWBI specifically) and a \ref Thread instance. It launches these threads and then basically just waits till it gets a kill (ctrl+c) command to close them down. Does a little keeping track of time as well.
 */
class Module: public yarp::os::RFModule
{
// DEFINE_CLASS_POINTER_TYPEDEFS(Module)

public:
    /*! Constructor which essentially does nothing.
     */
    Module();

    /*! Destructor which essentially does nothing.
     */
    ~Module();

    /*! Configures the module by parsing the RF contents.
     *  \param rf A resource finder instance which is initialized from the command line args.
     *
     *  \return True or false if the configuration was successful.
     */
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful

    /*! Interrupts the module execution and stops the control and wbi threads.
     */
    bool interruptModule(); // interrupt, e.g., the ports

    /*! Closes the module. First shuts down the threads.
     */
    bool close();

    /*! Updates the Module. Basically just clocks the thread run() method.
     *  \return Whether or not the clocking functions worked.
     */
    bool updateModule();

    /*! Prints all the command line args one could use.
     */
    void printHelp();

private:
    std::shared_ptr<Thread> ctrlThread; /*!< The controller thread. This is where the magic happens. */
    // Thread* ctrlThread; /*!< The controller thread. This is where the magic happens. */
    // Thread ctrlThread; /*!< The controller thread. This is where the magic happens. */
    std::shared_ptr<wbi::wholeBodyInterface> robotInterface; /*!< The yarpWBI interface used to get estimates from the robot. */

    yarp::os::Log yLog; /*!< A yarp logging tool. */
    OcraControllerOptions controller_options; /*!< Options used for the controller. */
    double avgTime; /*!< Average time between successive calls of the `run()` method.*/
    double stdDev; /*!< Standard deviation of the average time between successive calls of the `run()` method. */
    double avgTimeUsed; /*!< Average time for the `run()` method to execute. Should be close to avgTime. */
    double stdDevUsed; /*!< Standard deviation of the average time for the `run()` method to execute. */
    double dangerPeriodLoopTime; /*!< A value which the thread period loop time should not exceed. */
    static const int DEFAULT_THREAD_PERIOD = 10; /*!< If the user doesn't provide a thread period make it 10ms. */
};


#endif //OCRA_CONTROLLER_SERVER_MODULE_H
