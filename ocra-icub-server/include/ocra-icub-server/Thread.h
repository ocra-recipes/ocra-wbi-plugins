/*! \file       Thread.h
 *  \brief      The thread class for the controller server.
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

#ifndef OCRA_CONTROLLER_SERVER_THREAD_H
#define OCRA_CONTROLLER_SERVER_THREAD_H

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <wbi/wbi.h>

#include <ocra-icub-server/IcubControllerServer.h>

class OcraControllerOptions
{
// DEFINE_CLASS_POINTER_TYPEDEFS(OcraControllerOptions)

public: // Functions
    /*! Constructor. Initializes all of the possible values.
     */
    OcraControllerOptions();

    /*! Destructor. Does nothing.
     */
    ~OcraControllerOptions();

public: // Variables
    int                     threadPeriod; /*!< An int representing the looping period of the controller. */
    std::string             serverName; /*!< a string with the name of the controller server. */
    std::string             robotName; /*!< a string with the name of the robot. */
    std::string             startupTaskSetPath; /*!< a string with the absolute path to an xml file with a set of tasks. */
    std::string             startupSequence; /*!< a string with the name of a sequence to run **(will be removed)**. */
    std::string             wbiConfigFilePath; /*!< The absolute path to the configuration file used to initialize the yarpWBI. */
    bool                    runInDebugMode; /*!< a boolean which runs the controller in a debugging mode which allows one to check the contorller ouput joint by joint. */
    bool                    isFloatingBase; /*!< a boolean which tells the controller whether the robot has a fixed or floating base. */
    yarp::os::Property      yarpWbiOptions; /*!< Options for the WBI used to update the model. */
    ocra_recipes::CONTROLLER_TYPE    controllerType; /*!< The type of OCRA controller to use. */
};


/*! \class Thread
 *  \brief The meat and potatoes of the controller server.
 *
 *  \todo Remove task sequences.
 *  This class sets up an ocra::Model which is constructed from an OcraWbiModel and an ocra::Controller which can be  specified as either a WocraController, a GocraController, or a HocraController. The thread is looped at the period  specified by the user (defaults to 10ms) and on each loop the Model is updated and the control torques are  recalculated. *At the writing of this comment, task sequences are still in use and they too are initialized and  updated here. They will be removed eventually.*
 */
class Thread: public yarp::os::RateThread
{
// DEFINE_CLASS_POINTER_TYPEDEFS(Thread)
public:
    /*! Constructor
     *  \param controller_options The various arguments and options used to define what type of controller and tasks to use. See \ref OcraControllerOptions.
     *  \param wbi A shared pointer to a wholeBodyInterface object.
     */
    Thread(OcraControllerOptions& controller_options, std::shared_ptr<wbi::wholeBodyInterface> wbi);

    virtual ~Thread();
    bool threadInit();
    void run();
    void threadRelease();


private:
    IcubControllerServer ctrlServer;

    //TODO: Need to get the torque mins and maxs at the joint level.
    static const int TORQUE_MIN = -24; /*!< Minimum possible actuator torques */
    static const int TORQUE_MAX = 24; /*!< Maximum possible actuator torques */
    Eigen::ArrayXd minTorques; /*!< An eigen array filled with the min torques. */
    Eigen::ArrayXd maxTorques; /*!< An eigen array filled with the max torques. */


    OcraControllerOptions ctrlOptions; /*!< The controller options. */
    std::shared_ptr<wbi::wholeBodyInterface> robot; /*!< The WBI used to talk to the robot. */
    Eigen::VectorXd torques; /*!< The torques calculated at each run() loop. */
};


#endif // OCRA_CONTROLLER_SERVER_THREAD_H
