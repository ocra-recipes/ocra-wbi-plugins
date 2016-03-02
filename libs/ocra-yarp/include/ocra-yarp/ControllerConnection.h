/*! \file       ControllerConnection.h
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

#ifndef CONTROLLER_CONNECTION_H
#define CONTROLLER_CONNECTION_H

#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include "ocra/control/TaskManagers/TaskManagerMessageVocab.h"

#include "ocra-yarp/OcraYarpTools.h"
#include "ocra-yarp/OcraYarpVocab.h"

namespace ocra_yarp
{

class ControllerConnection
{
DEFINE_CLASS_POINTER_TYPEDEFS(ControllerConnection)

public:
    ControllerConnection();
    ~ControllerConnection();
    bool open(const bool openTaskPorts=true);
    void close();

    yarp::os::Bottle queryController(const OCRA_CONTROLLER_MESSAGE request);

    // void queryController(const OCRA_CONTROLLER_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTask(const std::string& taskName, const OCRA_CONTROLLER_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTask(const int taskIndex, const OCRA_CONTROLLER_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTasks(const OCRA_CONTROLLER_MESSAGE request, std::vector<yarp::os::Bottle&>& replies);
    // void queryTasks(const std::vector<OCRA_CONTROLLER_MESSAGE>& requests, std::vector<yarp::os::Bottle&>& replies);

    std::vector<std::string> getTaskPortNames();

private:



    yarp::os::RpcClient controllerRpcClient;
    std::vector<yarp::os::RpcClient*> taskRpcClients;

    yarp::os::Network yarp;


    bool connectToController(const std::string& controllerName = "Controller");
    bool connectToTaskPorts(const std::vector<std::string> taskPortNames);

    yarp::os::Log yLog;

    static int CONTROLLER_CONNECTION_COUNT;

    static constexpr double CONNECTION_TIMEOUT = 20.0;

};



} // namespace ocra_yarp
#endif // CONTROLLER_CONNECTION_H
