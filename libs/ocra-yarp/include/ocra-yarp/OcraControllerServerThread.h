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
#include "ocra-yarp/OcraYarpTools.h"

// #include "taskSequences/sequenceLibrary.h"

namespace ocra_yarp
{
class OcraControllerOptions
{
DEFINE_CLASS_POINTER_TYPEDEFS(OcraControllerOptions)

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
DEFINE_CLASS_POINTER_TYPEDEFS(OcraControllerServerThread)
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
    void parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply);
    std::string printValidMessageTags();

    /*! \class ControllerRpcServerCallback
     *  \brief A callback function which binds the rpc server port opened in the contoller server module to the controller thread's parsing function.
     */
    class ControllerRpcServerCallback : public yarp::os::PortReader
    {
    DEFINE_CLASS_POINTER_TYPEDEFS(ControllerRpcServerCallback)

    public:

        /*! Constructor
         *  \param ctThreadPtr A shared pointer to the control thread.
         */
        // ControllerRpcServerCallback(OcraControllerServerThread::shared_ptr ctThreadPtr);
        ControllerRpcServerCallback(OcraControllerServerThread& ctThreadPtr);

        /*! read
         *  \param connection Reads a port connection.
         *
         *  \return A boolean which tells whether or not a message was read.
         */
        virtual bool read(yarp::os::ConnectionReader& connection);

    private:

        // OcraControllerServerThread::shared_ptr ctThread; /*!< A shared pointer to the control thread. */
        OcraControllerServerThread& ctThread; /*!< A shared pointer to the control thread. */
    };


private:
    // Stabilization functions.
    bool loadStabilizationTasks();
    void stabilizeRobot();
    bool isRobotStable();

private:

    static const int ALL_JOINTS = -1; /*!< WBI all joints selector */

    //TODO: Need to get the torque mins and maxs at the joint level.
    static const int TORQUE_MIN = -24; /*!< Minimum possible actuator torques */
    static const int TORQUE_MAX = 24; /*!< Maximum possible actuator torques */
    Eigen::ArrayXd minTorques; /*!< An eigen array filled with the min torques. */
    Eigen::ArrayXd maxTorques; /*!< An eigen array filled with the max torques. */
    static constexpr double TIME_MSEC_TO_SEC = 0.001; /*!< For converting between milliseconds and seconds. */
    static constexpr double REFERENCE_JOINT_VELOCITY = 0.17; /*!< For converting between milliseconds and seconds. */


    OcraControllerOptions ctrlOptions; /*!< The controller options. */
    std::shared_ptr<wholeBodyInterface> robot; /*!< The WBI used to talk to the robot. */
    ocra::Controller *ctrl; /*!< The controller. */
    ocra::OneLevelSolverWithQuadProg internalSolver; /*!< The type of convex solver problem formulation used in the controller. */
    ocra::TaskSequence* taskSequence; /*!< The set of tasks currently being executed. */
    ocra::Model *ocraModel; /*!< The robot "model" which basically just provides state information to the controller. */
    OcraWbiModelUpdater::shared_ptr modelUpdater; /*!< A simple helper class which is called periodically to update the model by fetching state estimates from the WBI and passing them to Model. */


    OCRA_CONTROLLER_MESSAGE controllerStatus; /*!< The current status of the controller state. */


    int debugJointIndex; /*!< The current joint index being debugged. */

    Eigen::VectorXd torques; /*!< The torques calculated at each run() loop. */
    Eigen::VectorXd refSpeed; /*!< The reference joint velocity when in position mode control. */

    double time_sim; /*!< The relative time the controller has been looping. */
    double printPeriod; /*!< The frequency at which messages are printed. */
    double printCountdown;  /*!< When this hits 0.0 then a message is printed. */
    Eigen::VectorXd homePosture; /*!< The "home" posture of the robot. */
    Eigen::VectorXd initialPosture; /*!< The initial posture of the robot on controller startup. */
    Eigen::VectorXd debugPosture; /*!< A special posture used when debugging which solicits all joints to gravity. */
    Eigen::VectorXd measuredTorques; /*!< Gets torque estimates from the WBI durring debug mode. */




    // TODO: Convert to RPC port as below.
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort_in;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort_out;


    ControllerRpcServerCallback::shared_ptr rpcServerCallback; /*!< Rpc server port callback function. */
    yarp::os::RpcServer rpcServerPort; /*!< Rpc server port. */


private:
    // Stabilization variables.
    bool isStabilizing; /*!< Tells the run() loop to not update the task sequence when stablizing. */
    Eigen::Vector3d initialCoMPosition; /*!< The initial CoM position when the controller starts. We assume this is a stable CoM position. */
    Eigen::Vector3d initialTorsoPosition; /*!< The initial Torso position when the controller starts. We assume this is a stable Torso position. */

};

} // namespace ocra_yarp

#endif // OCRA_CONTROLLER_SERVER_THREAD_H
