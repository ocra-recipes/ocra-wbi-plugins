#ifndef CONTROLLERCONNECTION_H
#define CONTROLLERCONNECTION_H

#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>


#include <string>
#include <iostream>
#include <vector>


class controllerConnection
{

public:
    controllerConnection();
    ~controllerConnection();
    void open();
    void close();

    std::vector<std::string> getTaskPortNames();

private:


    yarp::os::Network yarp;
    yarp::os::RpcClient controllerRpcClient;

    std::vector<yarp::os::RpcClient*> taskRpcClients;

    bool connectToController(const std::string& controllerName = "wocraController");
    bool connectToTaskPorts(const std::vector<std::string> taskPortNames);



protected:

};


#endif
