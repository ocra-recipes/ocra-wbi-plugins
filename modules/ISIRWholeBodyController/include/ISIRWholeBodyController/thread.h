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

#ifndef ISIRWHOLEBODYCONTROLLERTHREAD_H
#define ISIRWHOLEBODYCONTROLLERTHREAD_H

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <wbi/wbi.h>

#include <ocraWbiPlugins/ocraWbiModel.h>

#include "wocra/wOcraController.h"
#include "wocra/Solvers/OneLevelSolver.h"
#include "wocra/Tasks/wOcraTaskSequenceBase.h"

#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace wbi;

namespace ISIRWholeBodyController
{

class ISIRWholeBodyControllerThread: public RateThread
{



    public:
        ISIRWholeBodyControllerThread(string _name,
                                      string _robotName,
                                      int _period,
                                      wholeBodyInterface *_wbi,
                                      yarp::os::Property & _options,
                                      string _startupTaskSetPath,
                                      string _startupSequence,
                                      bool _runInDebugMode,
                                      bool _isFreeBase);


        virtual ~ISIRWholeBodyControllerThread();
        bool threadInit();
        void run();
        void threadRelease();

        /** Start the controller. */
        void startController();


        /************** DataProcessor *************/
        class DataProcessor : public PortReader {
            private:
                ISIRWholeBodyControllerThread& ctThread;

            public:
                DataProcessor(ISIRWholeBodyControllerThread& ctThreadRef);

                virtual bool read(ConnectionReader& connection);
        };
        /************** DataProcessor *************/

    private:
        string name;
        string robotName;
        wholeBodyInterface *robot;
        ocraWbiModel *ocraModel;
        yarp::os::Property options;
        string startupTaskSetPath;
        string startupSequence;
        bool runInDebugMode;

        int debugJointIndex;

        wocra::wOcraController *ctrl;
        wocra::OneLevelSolverWithQuadProg internalSolver;

        wocra::wOcraTaskSequenceBase* taskSequence;

        Eigen::VectorXd q_initial; // stores vector with initial pose if we want to reset to this at the end

        // Member variables
        double time_sim;
        double printPeriod;
        double printCountdown;  // every time this is 0 (i.e. every printPeriod ms) print stuff
        Eigen::VectorXd fb_qRad; // vector that contains the encoders read from the robot
        Eigen::VectorXd fb_qdRad; // vector that contains the derivative of encoders read from the robot
        Eigen::VectorXd homePosture;
        Eigen::VectorXd debugPosture;
        Eigen::VectorXd refSpeed;

        // Eigen::VectorXd fb_Hroot_Vector;
        yarp::sig::Vector fb_Hroot_Vector;
        yarp::sig::Vector fb_Troot_Vector;
        yarp::sig::Vector torques_cmd;

        wbi::Frame fb_Hroot; // vector that position of root
        Eigen::Twistd fb_Troot; // vector that contains the twist of root
        yarp::sig::Vector fb_torque; // vector that contains the torque read from the robot

        // TODO: Convert to RPC port as below.
        yarp::os::BufferedPort<yarp::os::Bottle> debugPort_in;
        yarp::os::BufferedPort<yarp::os::Bottle> debugPort_out;

        bool usesYARP;
        RpcServer rpcPort;
        DataProcessor processor;

        void parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply);
        std::string printValidMessageTags();

};

} // end namespace

#endif
