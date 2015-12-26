#ifndef CONTROLLERCONNECTION_H
#define CONTROLLERCONNECTION_H value

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

    std::vector<std::string> getTaskPortNames();

private:

    std::vector<yarp::os::RpcClient*> taskRpcClients;

    yarp::os::Network yarp;
    yarp::os::RpcClient controllerRpcClient;


    bool connectToController(const std::string& controllerName = "wocraController");
    bool connectToTaskPorts(const std::vector<std::string> taskPortNames);



protected:

};


#endif
