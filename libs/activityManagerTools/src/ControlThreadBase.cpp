#include <activityManagerTools/ControlThreadBase.h>

int ControlThreadBase::threadId = 0;


ControlThreadBase::ControlThreadBase(int period, const std::string& taskRpcPortName):
RateThread(period),
taskRpcServerName(taskRpcPortName),
controlThreadPeriod(period),
weightDimension(0),
stateDimension(0),
closePortTimeout(5.0)
{
    ControlThreadBase::threadId++;
}

ControlThreadBase::~ControlThreadBase()
{

}


bool ControlThreadBase::threadInit()
{
    if(openControlPorts()){
        if(connectControlPorts()){
            if(getTaskDimensions()){
                getTaskParameters(originalTaskParams);
                currentTaskParams = originalTaskParams;
            }
        }
    }
    while(waitingForFirstStateMessage)
    {
        yarp::os::Time::delay(controlThreadPeriod/1000.);
    }
    return ct_threadInit();
}

void ControlThreadBase::threadRelease()
{
    std::cout << "\nControlThreadBase: Closing control ports for thread id = " << ControlThreadBase::threadId << " for task: " << originalTaskParams.name << ".\n\n";
    inputPort.close();
    outputPort.close();
    yarp::os::Bottle message, reply;
    message.addString("closeControlPorts");
    threadRpcClient.write(message, reply);
    double closeDelay = 0.5;
    double closeDelayTotal = 0.0;
    while (!reply.get(0).asInt() && closeDelayTotal < closePortTimeout) // if 1
    {
        reply.clear();
        threadRpcClient.write(message, reply);
        yarp::os::Time::delay(closeDelay);
        closeDelayTotal += closeDelay;
    }
    if (reply.get(0).asInt()) {
        threadRpcClient.close();
    }else{
        std::cout << "[WARNING](ControlThreadBase::threadRelease): Couldn't close task control ports." << std::endl;
    }
    std::cout << "Done.\n";


    ct_threadRelease();
}

void ControlThreadBase::run()
{
    ct_run();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ControlThreadBase::openControlPorts()
{
    bool portsOpened = yarp.checkNetwork();

    if (portsOpened)
    {
        std::string portNameBase = "/CT/" + getThreadType() + "/id_";
        std::stringstream portNameStream;
        portNameStream << ControlThreadBase::threadId;
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
        std::cout << "[ERROR](ControlThreadBase::openControlPorts): Yarp network not running." << std::endl;
    }

    return portsOpened;
}

bool ControlThreadBase::connectControlPorts()
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
        std::cout << "[ERROR](ControlThreadBase::connectControlPorts): Yarp network not running." << std::endl;
    }

    return portsConnected;
}

bool ControlThreadBase::parseInput(yarp::os::Bottle* input)
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
ControlThreadBase::inputCallback::inputCallback(ControlThreadBase& ctBaseRef):ctBase(ctBaseRef)
{
    //do nothing
}

bool ControlThreadBase::inputCallback::read(yarp::os::ConnectionReader& connection)
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


Eigen::VectorXd ControlThreadBase::getCurrentState()
{
    return currentStateVector;
}

void ControlThreadBase::sendGetStateMessage()
{
    yarp::os::Bottle message;
    message.addString("updateCurrentStateAndSend");
    inputPort.write(message);
}

bool ControlThreadBase::deactivateTask()
{
    yarp::os::Bottle message, reply;
    message.addString("deactivate");
    threadRpcClient.write(message, reply);
    if (reply.get(0).asString()=="deactivated")
    {
        getTaskParameters(currentTaskParams);
        return true;
    }
    else{return false;}
}

bool ControlThreadBase::activateTask()
{
    yarp::os::Bottle message, reply;
    message.addString("activate");
    threadRpcClient.write(message, reply);
    if (reply.get(0).asString()=="activated")
    {
        getTaskParameters(currentTaskParams);
        return true;
    }
    else{return false;}
}

bool ControlThreadBase::getTaskDimensions()
{
    yarp::os::Bottle message, reply;
    message.addString("getWeight");
    threadRpcClient.write(message, reply);
    if (reply.size()>1) {
        weightDimension = reply.size()-1;
    }else{
        std::cout << "[ERROR](ControlThreadBase::getTaskDimensions): Did not get a valid response from the task for its weight dimension." << std::endl;
        return false;
    }

    message.clear();
    reply.clear();
    message.addString("getDesired");
    threadRpcClient.write(message, reply);
    if (reply.size()>1) {
        stateDimension = reply.size()-1;
    }else{
        std::cout << "[ERROR](ControlThreadBase::getTaskDimensions): Did not get a valid response from the task for its state dimension." << std::endl;
        return false;
    }
    return true;
}

bool ControlThreadBase::getTaskParameters(TaskParameters& TP)
{
    TP.weight.resize(weightDimension);
    TP.desired.resize(stateDimension);
    TP.currentState.resize(stateDimension);


    yarp::os::Bottle message, reply;
    message.addString("getStiffness");
    message.addString("getDamping");
    message.addString("getDimension");
    message.addString("getWeight");
    message.addString("getDesired");
    message.addString("getCurrentState");
    message.addString("getType");
    message.addString("getName");
    message.addString("getActivityStatus");

    int maxCount = message.size();
    int count=0;

    threadRpcClient.write(message, reply);
    for(int i=0; i<reply.size(); i++)
    {
        if (reply.get(i).asString()=="Kp:"){
            i++;
            TP.kp = reply.get(i).asDouble();
            count++;
        }
        else if (reply.get(i).asString()=="Kd:"){
            i++;
            TP.kd = reply.get(i).asDouble();
            count++;
        }
        else if (reply.get(i).asString()=="Dimension:"){
            i++;
            TP.dimension = reply.get(i).asInt();
            count++;
        }
        else if (reply.get(i).asString()=="Weight:"){

            for(int j=0; j<weightDimension; j++){
                i++;
                TP.weight(j) = reply.get(i).asDouble();
            }
            count++;
        }
        else if (reply.get(i).asString()=="Desired:"){

            for(int j=0; j<stateDimension; j++){
                i++;
                TP.desired(j) = reply.get(i).asDouble();
            }
            count++;
        }
        else if (reply.get(i).asString()=="currentState:"){

            for(int j=0; j<stateDimension; j++){
                i++;
                TP.currentState(j) = reply.get(i).asDouble();
            }
            count++;
        }
        else if (reply.get(i).asString()=="Type:"){
            i++;
            TP.type = reply.get(i).asString();
            count++;
        }
        else if (reply.get(i).asString()=="Name:"){
            i++;
            TP.name = reply.get(i).asString();
            count++;
        }
        else if (reply.get(i).asString()=="activated"){
            i++;
            TP.isActive = reply.get(i).asBool();
            count++;
        }

    }
    return count==maxCount;

}
