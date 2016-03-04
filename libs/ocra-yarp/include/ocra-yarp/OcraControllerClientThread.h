/*! \file       OcraControllerClientThread.h
 *  \brief      A base class for all controller clients.
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

#ifndef OCRA_CONTROLLER_CLIENT_THREAD_H
#define OCRA_CONTROLLER_CLIENT_THREAD_H

#include <iostream>

#include <yarp/os/RateThread.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include "ocra-yarp/OcraYarpTools.h"
#include "ocra-yarp/ControlThread.h"
#include "ocra-yarp/TrajectoryThread.h"
#include "ocra-yarp/ControllerConnection.h"
#include "ocra-yarp/ModelThread.h"

namespace ocra_yarp
{
/*! \class OcraControllerClientThread
 *  \brief A RateThread implementation which provides a framework for ocra-controller-client threads.
 *
 *  The purpose of this class is to facilitate the construction of custom client threads by initializing many of the tools one could use in the production of a custom implementation. For example, a rpc port with callback is automatically opened, and the yarpWBI necessary for launching a \ref `ModelThread` is initialized. The idea here is that by simply inheriting this class you can get straight to opening up control threads and talking to tasks without having to worry about connecting to the controller and initializing stuff properly.
  * \note Make sure to implement at least the three pure virtual functions:
        - `virtual bool client_threadInit()=0;'
        - `virtual void client_threadRelease()=0;'
        - `virtual void client_run()=0;'
 */
class OcraControllerClientThread: public yarp::os::RateThread
{
DEFINE_CLASS_POINTER_TYPEDEFS(OcraControllerClientThread)

public:
    /*! Constructor
     *  \param period The desired looping time for the thread in ms.
     */
    OcraControllerClientThread(const int period = DEFAULT_CLIENT_THREAD_PERIOD);

    /*! Destructor, closes the rpc port.
     */
    ~OcraControllerClientThread();

    /*! Sets the whole body interface configuration.
     *  \param wbiOptions a Yarp Property object with the appropriate options for WBI
     */
    void setWbiOptions(yarp::os::Property& wbiOptions, const bool floatingBase);

    /*! Gets the thread period desired by the user.
     *  \return The thread period in ms.
     */
    int getExpectedPeriod();

    /*! Starts the internal model thread for users who need state info from the robot.
     *  \return True is model thread successfully starts.
     */
    bool startModelThread();

    /*! Stops the internal model thread.
     */
    void stopModelThread();

public: // public virtual functions

    /*! Gets the name of the thread. Override this function to set a different name.
     *  \return A string with the module name.
     */
    virtual std::string getThreadName();

protected: // protected virtual functions
    virtual void customCallbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply);

protected: // protected pure virtual functions
    virtual bool client_threadInit()=0;
    virtual void client_threadRelease()=0;
    virtual void client_run()=0;


protected: // RateThread pure virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

private:

    /*! Handles the incoming RPC messages. Makes sure they are not empty then passes them to \ref `customCallbackParser()`, which should be overloaded.
     *  \param message A bottle with the incoming message.
     *  \param reply A reply bottle to be filled in \ref `customCallbackParser()`.
     */
    void callbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply);



public:
    /*! \class threadCallback
     *  \brief A callback function which binds the rpc server port opened in the contoller server module to the controller thread's parsing function.
     */
    class threadCallback : public yarp::os::PortReader
    {
    DEFINE_CLASS_POINTER_TYPEDEFS(threadCallback)

    public:

        /*! Constructor
         *  \param newThreadRef A shared pointer to the control thread.
         */
        threadCallback(OcraControllerClientThread& newThreadRef);

        /*! read
         *  \param connection Reads a port connection.
         *
         *  \return A boolean which tells whether or not a message was read.
         */
        virtual bool read(yarp::os::ConnectionReader& connection);

    private:
        OcraControllerClientThread& threadRef;  /*!< A ref to the client thread. */
    };

private: // class variables

    std::shared_ptr<wbi::wholeBodyInterface> robotInterface;    /*!< The yarpWBI interface used to get estimates from the robot. */
    ControllerConnection::shared_ptr ctrlCon;                   /*!< The controller connection for the thread. */
    threadCallback::shared_ptr rpcCallback;                     /*!< Rpc server port callback function. */
    yarp::os::RpcServer rpcPort;                                /*!< Rpc server port. */
    yarp::os::Property yarpWbiOptions;
    yarp::os::Log yLog;                                         /*!< For logging in yarp. */
    static const int DEFAULT_CLIENT_THREAD_PERIOD = 10;         /*!< 10ms */
    static int CONTROLLER_CLIENT_THREAD_COUNT;                  /*!< A count that is incremented each time a client thread is constructed. */
    double expectedPeriod;                                      /*!< The user provided period. */
    std::string rpcPortName;                                    /*!< The name of the rpc port for the thread. */
    int threadNumber;                                           /*!< The unique thread number. */

    ModelThread::shared_ptr modelThread;                        /*!< A pointer to a model thread. */
    static const int MODEL_THREAD_PERIOD = 10;                  /*!< The period for the module thread in ms. */
    bool isFloatingBase;                                        /*!< Is the robot model floating or fixed base. */
};

} // namespace ocra_yarp

#endif // OCRA_CONTROLLER_CLIENT_H
