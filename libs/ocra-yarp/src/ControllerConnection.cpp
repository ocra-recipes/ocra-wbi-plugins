/*! \file       ControllerConnection.cpp
 *  \brief      A class for connecting to a running ocra-server.
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

#include <ocra-yarp/ControllerConnection.h>

using namespace ocra_yarp;

int ControllerConnection::CONTROLLER_CONNECTION_COUNT = 0;

ControllerConnection::ControllerConnection()
{
    controllerConnectionNumber = ++ControllerConnection::CONTROLLER_CONNECTION_COUNT;
}

ControllerConnection::~ControllerConnection()
{
    close();
}


bool ControllerConnection::open(const bool openTaskPorts, const std::string& connectionName)
{
    controllerConnectionName = connectionName;

    if (controllerConnectionName == "ControllerConnection_") {
        controllerConnectionName += std::to_string(controllerConnectionNumber);
    }

    bool res = true;
    std::cout << "Making controller connection..." << std::endl;
    res &= connectToController();
    if (res && openTaskPorts)
    {
        res &= connectToTaskPorts(getTaskPortNames());
        if(res)
        {
            std::cout << "Checking task manager rpc server connections..." << std::endl;
            for(auto rpc_i : taskRpcClients)
            {
                yarp::os::Bottle message, reply;
                message.addString("getType");
                rpc_i.second->write(message, reply);
                std::cout << reply.toString() << std::endl;
            }
            std::cout << "All set!" << std::endl;
        }else{
            yLog.error() << "Couldn't connect to the individual task ports.";
        }
    }
    return res;
}

void ControllerConnection::close()
{
    controllerRpcClient.close();
    for(auto rpc_i : taskRpcClients)
    {
        rpc_i.second->close();
    }
    taskRpcClients.clear();
}

void ControllerConnection::close(const std::string& taskName)
{
    if(taskRpcClients.find(taskName) != taskRpcClients.end())
    {
        taskRpcClients[taskName]->close();
        taskRpcClients.erase(taskName);
    }
    taskRpcClients.clear();
}

bool ControllerConnection::connectToController(const std::string& controllerName)
{
    if (!yarp.checkNetwork()) {
        yLog.error() << "Yarp network isn't running.";
        return false;
    }
    else{
        std::string controllerRpcClientName = "/OCRA/"+ controllerConnectionName +"/rpc:o";
        controllerRpcClient.open(controllerRpcClientName.c_str());
        std::string controllerListenerPortName = "/OCRA/"+ controllerConnectionName +":i";
        controllerListenerPort.open(controllerListenerPortName.c_str());
        rpcServerCallback = std::make_shared<ConnectionRpcServerCallback>(*this);
        controllerListenerPort.setReader(*rpcServerCallback);


        bool connected = false;
        double timeDelayed = 0.0;
        double delayTime = 1.0;
        while(!connected && timeDelayed < CONNECTION_TIMEOUT)
        {
            connected = yarp.connect(controllerRpcClientName.c_str(), "/OCRA/" + controllerName + "/rpc:i");
            yarp::os::Time::delay(delayTime);
            timeDelayed += delayTime;
            if (timeDelayed>= CONNECTION_TIMEOUT) {
                yLog.error() << "Could not connect to the ocra controller port. Are you sure it is running?";
            }
        }

        connected = false;
        timeDelayed = 0.0;
        while(!connected && timeDelayed < CONNECTION_TIMEOUT)
        {
            connected = yarp.connect("/OCRA/" + controllerName + ":o", controllerListenerPortName.c_str());
            yarp::os::Time::delay(delayTime);
            timeDelayed += delayTime;
            if (timeDelayed>= CONNECTION_TIMEOUT) {
                yLog.error() << "Could not connect to the ocra controller port. Are you sure it is running?";
            }
        }
        return connected;
    }

}


std::vector<std::string> ControllerConnection::getTaskPortNames()
{
    std::vector<std::string> portNameVec;
    yarp::os::Bottle message, reply;
    message.addInt(OCRA_CONTROLLER_MESSAGE::GET_TASK_PORT_LIST);
    controllerRpcClient.write(message, reply);
    for(auto i=0; i<reply.size(); ++i)
    {
        portNameVec.push_back(reply.get(i).asString());
    }
    return portNameVec;
}

bool ControllerConnection::connectToTaskPorts(const std::vector<std::string> taskPortNames)
{
    // taskRpcClients.resize(taskPortNames.size());
    bool taskConnected = true;

    // for(auto i=0; i<taskRpcClients.size(); ++i)
    for(auto i=0; i<taskPortNames.size(); ++i)
    {
        std::string tmpTaskName = taskPortNames[i].substr(3, taskPortNames[i].size()-1);
        std::string tmpTaskPortName = "/OCRA/" + controllerConnectionName;
        tmpTaskPortName += tmpTaskName;
        tmpTaskPortName += "o";
        taskRpcClients[tmpTaskName] = std::make_shared<yarp::os::RpcClient>();
        taskRpcClients[tmpTaskName]->open(tmpTaskPortName.c_str());
        taskConnected &= yarp.connect(tmpTaskPortName.c_str(), taskPortNames[i].c_str());
    }

    return taskConnected;
}

yarp::os::Bottle ControllerConnection::queryController(yarp::os::Bottle& requestBottle)
{
    yarp::os::Bottle reply;
    controllerRpcClient.write(requestBottle, reply);
    return reply;
}

yarp::os::Bottle ControllerConnection::queryController(const OCRA_CONTROLLER_MESSAGE request)
{
    yarp::os::Bottle requestBottle, reply;
    requestBottle.addInt(request);
    controllerRpcClient.write(requestBottle, reply);
    return reply;
}

yarp::os::Bottle ControllerConnection::queryController(const std::vector<OCRA_CONTROLLER_MESSAGE> requestVector)
{
    yarp::os::Bottle requestBottle, reply;
    for(auto request : requestVector){
        requestBottle.addInt(request);
    }
    controllerRpcClient.write(requestBottle, reply);
    return reply;
}

void ControllerConnection::parseControllerMessage(yarp::os::Bottle& input)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        // OCRA_CONTROLLER_MESSAGE message();
        switch (input.get(i).asInt()) {
            case REMOVE_TASK_PORT:
                {
                    ++i;
                    std::cout << "Got message: REMOVE_TASK_PORT." << input.get(i).asString() << std::endl;
                    close(input.get(i).asString());
                    ++i;
                }break;
        }
    }
}

/**************************************************************************************************
                                    Nested ConnectionRpcServerCallback Class
**************************************************************************************************/
ControllerConnection::ConnectionRpcServerCallback::ConnectionRpcServerCallback(ControllerConnection& parentConnectionRef)
: parentConnection(parentConnectionRef)
{
    //do nothing
}

bool ControllerConnection::ConnectionRpcServerCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input;

    if (!input.read(connection)){
        return false;
    }
    else{
        parentConnection.parseControllerMessage(input);
        // yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        // if (returnToSender!=NULL) {
        //     reply.write(*returnToSender);
        // }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/
