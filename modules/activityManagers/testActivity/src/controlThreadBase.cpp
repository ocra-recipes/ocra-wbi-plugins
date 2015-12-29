#include <testActivity/controlThreadBase.h>

int controlThreadBase::threadId = 0;


controlThreadBase::controlThreadBase(int period, const std::string& taskRpcPortName):
RateThread(period),
taskRpcServerName(taskRpcPortName),
controlThreadPeriod(period)
{
    controlThreadBase::threadId++;
}

controlThreadBase::~controlThreadBase()
{

}


bool controlThreadBase::threadInit()
{
    if(openControlPorts())
    {
        connectControlPorts();
    }
    while(waitingForFirstStateMessage)
    {
        yarp::os::Time::delay(controlThreadPeriod/1000.);
    }
    return ct_threadInit();
}

void controlThreadBase::threadRelease()
{
    std::cout << "controlThreadBase: Closing control ports for thread id = " << controlThreadBase::threadId << ".\n";
    inputPort.close();
    outputPort.close();
    yarp::os::Bottle message, reply;
    message.addString("closeControlPorts");
    threadRpcClient.write(message, reply);
    if (reply.get(0).asInt()) // if 1
    {
        threadRpcClient.close();
    }
    std::cout << "Done.";


    ct_threadRelease();
}

void controlThreadBase::run()
{
    ct_run();
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

        isFirstInputBottle = true;
        inpCallback = new inputCallback(*this);
        inputPort.setReader(*inpCallback);

        waitingForFirstStateMessage = true;

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

bool controlThreadBase::parseInput(yarp::os::Bottle* input)
{
    if (isFirstInputBottle) {
        currentStateVector.resize(input->size());
        isFirstInputBottle = false;
        waitingForFirstStateMessage = false;
    }
    for(int i=0; i<input->size(); i++)
    {
        currentStateVector(i) = input->get(i).asDouble();
    }
}


/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
controlThreadBase::inputCallback::inputCallback(controlThreadBase& ctBaseRef):ctBase(ctBaseRef)
{
    //do nothing
}

bool controlThreadBase::inputCallback::read(yarp::os::ConnectionReader& connection)
{
    // std::cout << "Got a message!" << std::endl;
    yarp::os::Bottle input;
    if (input.read(connection)){
        return ctBase.parseInput(&input);
    }
    else{
        return false;
    }
}
/**************************************************************************************************
**************************************************************************************************/


Eigen::VectorXd controlThreadBase::getCurrentState()
{
    // sendGetStateMessage();
    return currentStateVector;
}

void controlThreadBase::sendGetStateMessage()
{
    yarp::os::Bottle message;
    message.addString("updateCurrentStateAndSend");
    inputPort.write(message);
}
