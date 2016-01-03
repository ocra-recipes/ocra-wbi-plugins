#include <testActivity/ControllerConnection.h>

#ifndef CONNECTION_TIMEOUT
#define CONNECTION_TIMEOUT 20.0
#endif

ControllerConnection::ControllerConnection()
{
    open();
}

ControllerConnection::~ControllerConnection()
{
    close();
}


void ControllerConnection::open()
{
    std::cout << "Making controller connection..." << std::endl;
    if (connectToController()) {
        if(connectToTaskPorts(getTaskPortNames()))
        {
            std::cout << "Checking task manager rpc server connections..." << std::endl;
            for(int i=0; i<taskRpcClients.size(); i++)
            {
                yarp::os::Bottle message, reply;
                message.addString("getType");
                taskRpcClients[i]->write(message, reply);
                std::cout << reply.toString() << std::endl;
            }
        }else{
                //error message
        }
    }else{
        //error message
    }
}

void ControllerConnection::close()
{
    controllerRpcClient.close();
    for(int i=0; i<taskRpcClients.size(); i++)
    {
        taskRpcClients[i]->close();
    }
}

bool ControllerConnection::connectToController(const std::string& controllerName)
{
    if (!yarp.checkNetwork()) {
        std::cout << "[ERROR](ControllerConnection::connectToController): Yarp network isn't running." << std::endl;
        return false;
    }
    else{
        std::string controllerRpcClientName = "/CC/" + controllerName + "/rpc:o";
        controllerRpcClient.open(controllerRpcClientName.c_str());
        bool connected = false;
        double timeDelayed = 0.0;
        double delayTime = 1.0;
        while(!connected && timeDelayed < CONNECTION_TIMEOUT)
        {
            connected = yarp.connect(controllerRpcClientName.c_str(), "/" + controllerName + "/rpc:i");
            yarp::os::Time::delay(delayTime);
            timeDelayed += delayTime;
            if (timeDelayed>= CONNECTION_TIMEOUT) {
                std::cout << "[ERROR_TIMEOUT](ControllerConnection::connectToController): Could not connect to " <<controllerName << "." << std::endl;
            }
        }
        return connected;
    }

}


std::vector<std::string> ControllerConnection::getTaskPortNames()
{
    std::vector<std::string> portNameVec;
    yarp::os::Bottle message, reply;
    message.addString("getTaskPorts");
    controllerRpcClient.write(message, reply);
    for(int i=1; i<reply.size(); i++)
    {
        portNameVec.push_back(reply.get(i).asString());
    }
    return portNameVec;
}

bool ControllerConnection::connectToTaskPorts(const std::vector<std::string> taskPortNames)
{
    int numberOfTasks = taskPortNames.size();
    taskRpcClients.resize(numberOfTasks);
    bool taskConnected = true;

    for(int i=0; i<numberOfTasks; i++)
    {
        std::string tmpTaskPortName = "/CC";
        tmpTaskPortName += taskPortNames[i].substr(3, taskPortNames[i].size()-1);
        tmpTaskPortName += "o";
        taskRpcClients[i] = new yarp::os::RpcClient;
        taskRpcClients[i]->open(tmpTaskPortName.c_str());
        taskConnected = taskConnected && yarp.connect(tmpTaskPortName.c_str(), taskPortNames[i].c_str());
    }

    return taskConnected;
}
