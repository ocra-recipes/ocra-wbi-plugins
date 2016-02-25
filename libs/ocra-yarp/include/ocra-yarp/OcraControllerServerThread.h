/*! \file       OcraControllerServerThread.h
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

#include <cmath>
#include <iostream>
#include <memory>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/Property.h>


#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <wbi/wbi.h>

#include "ocra/control/Controller.h"
#include "wocra/WocraController.h"
#include "ocra/optim/OneLevelSolver.h"
#include "ocra/control/TaskManagers/TaskSequence.h"
#include "ocra/control/TaskManagers/TaskParser.h"

#include "ocra-yarp/OcraWbiModel.h"
#include "ocra-yarp/OcraWbiConversions.h"
#include "ocra-yarp/OcraWbiModelUpdater.h"
#include "ocra-yarp/OcraYarpVocab.h"


#include "taskSequences/sequenceLibrary.h"

namespace ocra_yarp
{
class OcraControllerOptions
{
public: // Functions
    /*! Constructor. Initializes all of the possible values.
     */
    OcraControllerOptions()
    : threadPeriod(10)
    , serverName("")
    , robotName("")
    , startupTaskSetPath("")
    , startupSequence("")
    , runInDebugMode(false)
    , isFloatingBase(false)
    , yarpWbiOptions(yarp::os::Property())
    , controllerType(WOCRA_CONTROLLER)
    {
    }

    /*! Destructor. Does nothing.
     */
    ~OcraControllerOptions(){};

public: // Variables
    int                     threadPeriod; /*!< An int representing the looping period of the controller. */
    std::string             serverName; /*!< a string with the name of the controller server. */
    std::string             robotName; /*!< a string with the name of the robot. */
    std::string             startupTaskSetPath; /*!< a string with the absolute path to an xml file with a set of tasks. */
    std::string             startupSequence; /*!< a string with the name of a sequence to run **(will be removed)**. */
    bool                    runInDebugMode; /*!< a boolean which runs the controller in a debugging mode which allows one to check the contorller ouput joint by joint. */
    bool                    isFloatingBase; /*!< a boolean which tells the controller whether the robot has a fixed or floating base. */
    yarp::os::Property      yarpWbiOptions; /*!< Options for the WBI used to update the model. */
    OCRA_CONTROLLER_TYPE    controllerType; /*!< The type of OCRA controller to use. */
};


/*! \class OcraControllerServerThread
 *  \brief The meat and potatoes of the controller server.
 *
 *  \todo Remove task sequences.
 *  This class sets up an ocra::Model which is constructed from an OcraWbiModel and an ocra::Controller which can be  specified as either a WocraController, a GocraController, or a HocraController. The thread is looped at the period  specified by the user (defaults to 10ms) and on each loop the Model is updated and the control torques are  recalculated. *At the writing of this comment, task sequences are still in use and they too are initialized and  updated here. They will be removed eventually.*
 */
class OcraControllerServerThread: public yarp::os::RateThread
{
public:

    /*! Constructor
     *  \param controller_options The various arguments and options used to define what type of controller and tasks to use. See \ref OcraControllerOptions.
     *  \param wbi A shared pointer to a wholeBodyInterface object.
     */
    OcraControllerServerThread(OcraControllerOptions& controller_options, std::shared_ptr<wbi::wholeBodyInterface> wbi);

    virtual ~OcraControllerServerThread();
    bool threadInit();
    void run();
    void threadRelease();

    /************** DataProcessor *************/
    class DataProcessor : public yarp::os::PortReader {
        private:
            OcraControllerServerThread& ctThread;

        public:
            DataProcessor(OcraControllerServerThread& ctThreadRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
    };
    /************** DataProcessor *************/

private:
    OcraControllerOptions ctrlOptions;
    std::shared_ptr<wholeBodyInterface> robot;

    ocra::Controller *ctrl;
    ocra::OneLevelSolverWithQuadProg internalSolver;

    ocra::TaskSequence* taskSequence;
    ocra::Model *ocraModel;
    OcraWbiModelUpdater* modelUpdater;

    bool loadStabilizationTasks();
    void stabilizeRobot();
    bool isRobotStable();
    bool isStabilizing;


    int debugJointIndex;


    Eigen::VectorXd q_initial; // stores vector with initial pose if we want to reset to this at the end

    // Member variables
    double time_sim;
    double printPeriod;
    double printCountdown;  // every time this is 0 (i.e. every printPeriod ms) print stuff
    Eigen::VectorXd homePosture;
    Eigen::VectorXd initialPosture;
    Eigen::VectorXd debugPosture;
    Eigen::VectorXd refSpeed;
    Eigen::Vector3d initialCoMPosition;
    Eigen::Vector3d initialTorsoPosition;


    yarp::sig::Vector torques_cmd;
    yarp::sig::Vector fb_torque; // vector that contains the torque read from the robot


    // TODO: Convert to RPC port as below.
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort_in;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort_out;




    bool usesYARP;
    yarp::os::RpcServer rpcPort;
    DataProcessor processor;

    void parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply);
    std::string printValidMessageTags();

};

} // namespace ocra_yarp

#endif // OCRA_CONTROLLER_SERVER_THREAD_H
