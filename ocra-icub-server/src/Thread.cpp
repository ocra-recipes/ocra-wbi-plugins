/*! \file       Thread.cpp
 *  \brief      The thread class for the controller server.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-icub.
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

#include "ocra-icub-server/Thread.h"





OcraControllerOptions::OcraControllerOptions()
: threadPeriod(10)
, serverName("")
, robotName("")
, startupTaskSetPath("")
, startupSequence("")
, wbiConfigFilePath("")
, runInDebugMode(false)
, isFloatingBase(false)
, yarpWbiOptions(yarp::os::Property())
, controllerType(ocra_recipes::WOCRA_CONTROLLER)
, solver(ocra_recipes::QUADPROG)
{
}

OcraControllerOptions::~OcraControllerOptions()
{
}


std::ostream& operator<<(std::ostream &out, const OcraControllerOptions& opts)
{
    out << "----------------------\n" << "Controller Options" << "\n----------------------\n";
    out << "threadPeriod: " << opts.threadPeriod << "\n\n";
    out << "serverName: " << opts.serverName << "\n\n";
    out << "robotName: " << opts.robotName << "\n\n";
    out << "startupTaskSetPath: " << opts.startupTaskSetPath << "\n\n";
    out << "startupSequence: " << opts.startupSequence << "\n\n";
    out << "wbiConfigFilePath: " << opts.wbiConfigFilePath << "\n\n";
    out << "runInDebugMode: " << opts.runInDebugMode << "\n\n";
    out << "isFloatingBase: " << opts.isFloatingBase << "\n\n";
    // out << "yarpWbiOptions: " << opts.yarpWbiOptions << "\n\n";
    out << "controllerType: " << opts.controllerType << "\n\n";
    out << "solver: " << opts.solver << "\n\n";

    return out;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Constructor
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
Thread::Thread(OcraControllerOptions& controller_options, std::shared_ptr<wbi::wholeBodyInterface> wbi)
: RateThread(controller_options.threadPeriod)
, ctrlOptions(controller_options)
, controllerStatus(ocra_icub::CONTROLLER_SERVER_STOPPED)
{
    std::cout << ctrlOptions << std::endl;

    yarpWbi = wbi;
    bool usingInterprocessCommunication = true;
    ctrlServer = std::make_shared<IcubControllerServer>(  yarpWbi,
                                        ctrlOptions.robotName,
                                        ctrlOptions.isFloatingBase,
                                        ctrlOptions.controllerType,
                                        ctrlOptions.solver,
                                        usingInterprocessCommunication);

    ctrlServer->initialize();

    model = ctrlServer->getRobotModel();
    // Construct rpc server callback and bind to the control thread.
    rpcServerCallback = std::make_shared<ControllerRpcServerCallback>(*this);
    // Open the rpc server port.
    rpcServerPort.open("/ocra-icub-server/info/rpc:i");
    // Bind the callback to the port.
    rpcServerPort.setReader(*rpcServerCallback);


}

Thread::~Thread()
{
    rpcServerPort.close();
    if(ctrlOptions.runInDebugMode) {
        debugRpcPort.close();
        debugRefOutPort.close();
        debugRealOutPort.close();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Init
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thread::threadInit()
{
    // TODO: Add a check to make sure the tasks get loaded in and if not - don't change the control mode. return false;
    ctrlServer->addTaskManagersFromXmlFile(ctrlOptions.startupTaskSetPath);
    minTorques      = Eigen::ArrayXd::Constant(yarpWbi->getDoFs(), TORQUE_MIN);
    maxTorques      = Eigen::ArrayXd::Constant(yarpWbi->getDoFs(), TORQUE_MAX);
    initialPosture  = Eigen::VectorXd::Zero(yarpWbi->getDoFs());
    yarpWbi->getEstimates(wbi::ESTIMATE_JOINT_POS, initialPosture.data(), ALL_JOINTS);
    controllerStatus = ocra_icub::CONTROLLER_SERVER_RUNNING;
    if(ctrlOptions.runInDebugMode) {
        debugJointIndex = 0;
        debuggingAllJoints = false;
        std::string debugRpcPortName("/ocra-icub-server/debug/rpc:i");
        std::string debugRefOutPortName("/ocra-icub-server/debug/ref:o");
        std::string debugRealOutPortName("/ocra-icub-server/debug/real:o");


        debugRpcPort.open(debugRpcPortName);
        debugRpcCallback = std::make_shared<DebugRpcServerCallback>(*this);
        debugRpcPort.setReader(*debugRpcCallback);

        debugRefOutPort.open(debugRefOutPortName);
        debugRealOutPort.open(debugRealOutPortName);
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "\t--> Running in DEBUG mode <--" << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "-- RPC port open at: " << debugRpcPortName << std::endl;
        std::cout << "-- Output port open at: " << debugRefOutPortName << std::endl;
        std::cout << "-- Input port open at: " << debugRealOutPortName << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::string jointName = model->getJointName(debugJointIndex);
        std::cout << "Debugging joint index: " << debugJointIndex << " ("<< jointName <<")" << std::endl;


        yarpWbi->setControlMode(wbi::CTRL_MODE_POS, initialPosture.data(), ALL_JOINTS);
        yarpWbi->setControlReference(initialPosture.data());
        return yarpWbi->setControlMode(wbi::CTRL_MODE_TORQUE, 0, debugJointIndex);
    } else {
        return yarpWbi->setControlMode(wbi::CTRL_MODE_TORQUE, 0, ALL_JOINTS);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                               Thread Run
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Thread::run()
{
	ctrlServer->computeTorques(torques);
    torques = ((torques.array().max(minTorques)).min(maxTorques)).matrix().eval();

    if (ctrlOptions.runInDebugMode) {
        measuredTorques = model->getJointTorques();
        writeDebugData();
        if (debuggingAllJoints) {
            yarpWbi->setControlReference(torques.data());
        } else {
            double refTau = torques(debugJointIndex);
            yarpWbi->setControlReference(&refTau, debugJointIndex);
        }
    } else {
        yarpWbi->setControlReference(torques.data());
    }

}

void Thread::threadRelease()
{
    controllerStatus = ocra_icub::CONTROLLER_SERVER_STOPPED;

    yarpWbi->setControlMode(wbi::CTRL_MODE_POS, initialPosture.data(), ALL_JOINTS);
    yarpWbi->setControlReference(initialPosture.data());

}

void Thread::writeDebugData()
{
    yarp::os::Bottle refBottle, realBottle;
    for (int i=0; i<torques.size(); ++i) {
        refBottle.addDouble(torques(i));
        realBottle.addDouble(measuredTorques(i));
    }
    debugRefOutPort.write(refBottle);
    debugRealOutPort.write(realBottle);
}


void Thread::parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        // OCRA_CONTROLLER_MESSAGE message();
        switch (input.get(i).asInt()) {
            case ocra_icub::GET_CONTROLLER_SERVER_STATUS:
                {
                    std::cout << "Got message: GET_CONTROLLER_SERVER_STATUS." << std::endl;
                    reply.addInt(controllerStatus);
                    ++i;
                }break;

            case ocra_icub::GET_MODEL_CONFIG_INFO:
                {
                    std::cout << "Got message: GET_MODEL_CONFIG_INFO." << std::endl;
                    reply.addString(ctrlOptions.wbiConfigFilePath);
                    reply.addString(ctrlOptions.robotName);
                    reply.addInt(ctrlOptions.isFloatingBase);
                    ++i;
                }break;

            case ocra_icub::HELP:
                {
                    ++i;
                    std::cout << "Got message: HELP." << std::endl;
                }break;

            default:
                {
                    ++i;
                    std::cout << "Got message: UNKNOWN." << std::endl;
                }break;

        }
    }
}

void Thread::parseDebugMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize; ++i)
    {
        std::string key = input.get(i).asString();
        if (key == "setJoint") {
            std::string jointName;
            std::string jointString;
            std::string replyString;
            int newIndex = input.get(++i).asInt();
            if (newIndex == -1) {
                if(yarpWbi->setControlMode(wbi::CTRL_MODE_TORQUE, torques.data(), ALL_JOINTS) ) {
                    debuggingAllJoints = true;
                    replyString = "Success! Setting all joints to TORQUE control mode.";
                } else {
                    replyString = "FAILED! Could not set the control mode of the joints to TORQUE mode.";
                }
            } else {
                jointName = model->getJointName(newIndex);
                jointString = std::to_string(newIndex);
                if (newIndex >= 0 && newIndex < initialPosture.rows()) {
                    if (debuggingAllJoints) {
                        yarpWbi->setControlMode(wbi::CTRL_MODE_POS, initialPosture.data(), ALL_JOINTS);
                        yarpWbi->setControlReference(initialPosture.data(), ALL_JOINTS);
                        debuggingAllJoints = false;
                    } else {
                        yarpWbi->setControlMode(wbi::CTRL_MODE_POS, &initialPosture(debugJointIndex), debugJointIndex);
                        yarpWbi->setControlReference(&initialPosture(debugJointIndex), debugJointIndex);
                    }

                    if(yarpWbi->setControlMode(wbi::CTRL_MODE_TORQUE, &torques(newIndex), newIndex) ) {
                        replyString = "Success! Debugging joint index: " + jointString + " (" + jointName +")";
                    } else {
                        replyString = "FAILED! Could not set the control mode of joint " + jointString + " ("+jointName+") to TORQUE mode.";
                    }
                    debugJointIndex = newIndex;
                } else {
                    replyString = "FAILED! The index " + jointString + " is outside of the valid range, [0-" + std::to_string(initialPosture.rows() - 1)+ "] Use index = -1 for all joints. Type [listJoints] or [help] for details.";
                }
            }
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString(replyString);
            std::cout << replyString << std::endl;

        } else if (key == "listJoints") {
            std::string replyString("Joint List:\n");
            reply.addVocab(yarp::os::Vocab::encode("many"));
            for (int i=0; i<initialPosture.rows(); ++i) {
                wbi::ID dofID;
                yarpWbi->getJointList().indexToID(i, dofID);
                std::string tmp = std::to_string(i) + " : " + dofID.toString();
                replyString += tmp + "\n";
                reply.addString(tmp);
            }

            std::cout << replyString << std::endl;

        } else if (key == "help") {
            std::string helpString("");
            helpString += "Valid commands: \n";
            helpString += "-- setJoint [index]\n";
            helpString += "-- listJoints\n";
            helpString += "-- help\n";
            std::cout << helpString << std::endl;
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString(helpString);
        } else {
            reply.addString("Unknown command. Type [help] for usage details.");
        }
    }
}

/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
Thread::ControllerRpcServerCallback::ControllerRpcServerCallback(Thread& threadRef)
: thread(threadRef)
{
    //do nothing
}

bool Thread::ControllerRpcServerCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        thread.parseIncomingMessage(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/

/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
Thread::DebugRpcServerCallback::DebugRpcServerCallback(Thread& threadRef)
: thread(threadRef)
{
    //do nothing
}

bool Thread::DebugRpcServerCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        thread.parseDebugMessage(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/
