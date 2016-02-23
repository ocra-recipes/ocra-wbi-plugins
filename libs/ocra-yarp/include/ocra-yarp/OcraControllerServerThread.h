#ifndef OCRA_CONTROLLER_SERVER_THREAD_H
#define OCRA_CONTROLLER_SERVER_THREAD_H

#include <cmath>
#include <iostream>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <wbi/wbi.h>

#include "ocra/control/Controller.h"
#include "wocra/WocraController.h"
#include "ocra/optim/OneLevelSolver.h"
#include "ocra/control/TaskManagers/TaskSequence.h"
#include "ocra/control/TaskManagers/TaskParser.h"

#include "ocra-yarp/OcraWbiModel.h"
#include "ocra-yarp/OcraWbiConversions.h"

#include "taskSequences/sequenceLibrary.h"

namespace ocra_yarp
{

class OcraControllerServerThread: public yarp::os::RateThread
{



    public:
        OcraControllerServerThread(   std::string _name,
                                      std::string _robotName,
                                      int _period,
                                      wholeBodyInterface *_wbi,
                                      yarp::os::Property & _options,
                                      std::string _startupTaskSetPath,
                                      std::string _startupSequence,
                                      bool _runInDebugMode,
                                      bool _isFreeBase);


        virtual ~OcraControllerServerThread();
        bool threadInit();
        void run();
        void threadRelease();

        /** Start the controller. */
        void startController();


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

        bool loadStabilizationTasks();
        void stabilizeRobot();
        bool isRobotStable();
        bool isStabilizing;

        std::string name;
        std::string robotName;
        wholeBodyInterface *robot;
        OcraWbiModel *ocraModel;
        yarp::os::Property options;
        std::string startupTaskSetPath;
        std::string startupSequence;
        bool runInDebugMode;

        int debugJointIndex;


        ocra::Controller *ctrl;
        ocra::OneLevelSolverWithQuadProg internalSolver;

        ocra::TaskSequence* taskSequence;

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
        yarp::os::RpcServer rpcPort;
        DataProcessor processor;

        void parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply);
        std::string printValidMessageTags();

};

} // end namespace

#endif
