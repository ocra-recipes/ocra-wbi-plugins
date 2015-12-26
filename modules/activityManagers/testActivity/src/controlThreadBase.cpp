#include <testActivity/controlThreadBase.h>

int controlThreadBase::threadId = 0;


controlThreadBase::controlThreadBase(int period, const std::string& taskRpcPortName):
RateThread(period),
taskRpcServerName(taskRpcPortName)
{
    controlThreadType = "controlThreadBase";
    controlThreadBase::threadId++;
    if(openControlPorts())
    {
        connectControlPorts();
    }

}

controlThreadBase::~controlThreadBase()
{
    std::cout << "controlThreadBase: Closing control ports...\n";
    inputPort.close();
    outputPort.close();
    std::cout << "Done.";
}


bool controlThreadBase::threadInit()
{
    return true;
}

void controlThreadBase::threadRelease()
{
    // Nothing here.
}

void controlThreadBase::run()
{
    // Nothing here.
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool controlThreadBase::openControlPorts()
{
    bool portsOpened = yarp.checkNetwork();

    if (portsOpened)
    {
        std::string portNameBase = "/CT/" + getThreadType() + "/id_";
        std::stringstream portNameStream;
        portNameStream << controlThreadBase::threadId;
        portNameBase += portNameStream.str();

        inputPortName = portNameBase + ":i";
        outputPortName = portNameBase + ":o";


        portsOpened = portsOpened && inputPort.open(inputPortName.c_str());
        portsOpened = portsOpened && outputPort.open(outputPortName.c_str());

        threadRpcClientName = portNameBase + "/rpc:o";

        portsOpened = portsOpened && threadRpcClient.open(threadRpcClientName.c_str());


    }
    else{
        std::cout << "[ERROR](controlThreadBase::openControlPorts): Yarp network not running." << std::endl;
    }

    return portsOpened;
}

bool controlThreadBase::connectControlPorts()
{
    bool portsConnected = yarp.checkNetwork();

    if (portsConnected)
    {
        portsConnected = portsConnected && yarp.connect(threadRpcClientName.c_str(), taskRpcServerName.c_str());

        yarp::os::Bottle message, reply;
        message.addString("openControlPorts");
        threadRpcClient.write(message, reply);
        if (reply.get(0).asInt()) // if 1
        {
            std::string taskOutputPortName, taskInputPortName;

            message.clear();
            reply.clear();
            message.addString("getControlPortNames");
            threadRpcClient.write(message, reply);

            taskInputPortName = reply.get(1).asString();
            taskOutputPortName = reply.get(2).asString();

            portsConnected = portsConnected && yarp.connect(taskOutputPortName.c_str(), inputPortName.c_str());
            portsConnected = portsConnected && yarp.connect(outputPortName.c_str(), taskInputPortName.c_str());

        }
        else
        {
            portsConnected = false;
        }
    }
    else{
        std::cout << "[ERROR](controlThreadBase::connectControlPorts): Yarp network not running." << std::endl;
    }

    return portsConnected;
}

std::string controlThreadBase::getThreadType()
{
    return controlThreadType;
}
