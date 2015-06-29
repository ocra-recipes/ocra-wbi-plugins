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
//#include "wOcraCtrlTaskManager.h"
// #include <ISIRWholeBodyController/sequenceCollection.h>



#include "wocra/wOcraController.h"
#include "wocra/Solvers/OneLevelSolver.h"
#include "wocra/Tasks/wOcraTaskSequenceBase.h"

// #include "ISIRWholeBodyController/ScenariosICub.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace wbi;

namespace ISIRWholeBodyController
{

class ISIRWholeBodyControllerThread: public RateThread
{
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

    wocra::wOcraTaskSequenceBase* baseSequence;
    wocra::wOcraTaskSequenceBase* xmlSequence;
    wocra::wOcraTaskSequenceBase* cppSequence;

    bool baseSequenceIsActive, xmlSequenceIsActive, cppSequenceIsActive;

    Eigen::VectorXd q_initial; // stores vector with initial pose if we want to reset to this at the end

    // Member variables
    double time_sim;
    double printPeriod;
    double printCountdown;  // every time this is 0 (i.e. every printPeriod ms) print stuff
    Eigen::VectorXd fb_qRad; // vector that contains the encoders read from the robot
    Eigen::VectorXd fb_qdRad; // vector that contains the derivative of encoders read from the robot
    Eigen::VectorXd homePosture;
    Eigen::VectorXd debugPosture;

    // Eigen::VectorXd fb_Hroot_Vector;
    yarp::sig::Vector fb_Hroot_Vector;
    yarp::sig::Vector fb_Troot_Vector;
    yarp::sig::Vector torques_cmd;

    wbi::Frame fb_Hroot; // vector that position of root
    Eigen::Twistd fb_Troot; // vector that contains the twist of root
    yarp::sig::Vector fb_torque; // vector that contains the torque read from the robot


    yarp::os::BufferedPort<yarp::os::Bottle> debugPort_in;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort_out;

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

    bool threadInit();
    void run();
    void threadRelease();

    /** Start the controller. */
    void startController();

};

} // end namespace

#endif
