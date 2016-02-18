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

#ifndef WOCRACONTROLLER_THREAD_H
#define WOCRACONTROLLER_THREAD_H


#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>


#include <wbi/wbi.h>

#include <ocraWbiPlugins/ocraWbiModel.h>


#include "ocra/control/Controller.h"
#include "wocra/wOcraController.h"
#include "ocra/optim/OneLevelSolver.h"
#include "wocra/Tasks/wOcraTaskSequenceBase.h"


#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>

#include <cmath>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace wbi;

namespace wocraController
{

class wocraControllerThread: public RateThread
{



    public:
        wocraControllerThread(string _name,
                                      string _robotName,
                                      int _period,
                                      wholeBodyInterface *_wbi,
                                      yarp::os::Property & _options,
                                      string _startupTaskSetPath,
                                      string _startupSequence,
                                      bool _runInDebugMode,
                                      bool _isFreeBase);


        virtual ~wocraControllerThread();
        bool threadInit();
        void run();
        void threadRelease();

        /** Start the controller. */
        void startController();


        /************** DataProcessor *************/
        class DataProcessor : public PortReader {
            private:
                wocraControllerThread& ctThread;

            public:
                DataProcessor(wocraControllerThread& ctThreadRef);

                virtual bool read(ConnectionReader& connection);
        };
        /************** DataProcessor *************/

    private:

        bool loadStabilizationTasks();
        void stabilizeRobot();
        bool isRobotStable();
        bool isStabilizing;

        string name;
        string robotName;
        wholeBodyInterface *robot;
        ocraWbiModel *ocraModel;
        yarp::os::Property options;
        string startupTaskSetPath;
        string startupSequence;
        bool runInDebugMode;

        int debugJointIndex;


        ocra::Controller *ctrl;
        ocra::OneLevelSolverWithQuadProg internalSolver;

        wocra::wOcraTaskSequenceBase* taskSequence;

        Eigen::VectorXd q_initial; // stores vector with initial pose if we want to reset to this at the end

        // Member variables
        double time_sim;
        double printPeriod;
        double printCountdown;  // every time this is 0 (i.e. every printPeriod ms) print stuff
        Eigen::VectorXd fb_qRad; // vector that contains the encoders read from the robot
        Eigen::VectorXd fb_qdRad; // vector that contains the derivative of encoders read from the robot
        Eigen::VectorXd homePosture;
        Eigen::VectorXd initialPosture;
        Eigen::VectorXd debugPosture;
        Eigen::VectorXd refSpeed;
        Eigen::Vector3d initialCoMPosition;
        Eigen::Vector3d initialTorsoPosition;


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
