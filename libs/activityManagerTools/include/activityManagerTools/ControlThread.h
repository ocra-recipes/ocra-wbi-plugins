#ifndef CONTROLTHREAD_H
#define CONTROLTHREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>


#include <string>
#include <sstream>
#include <iostream>

#include <Eigen/Dense>

class TaskParameters
{
public:
    double kp;
    double kd;
    int dimension;
    Eigen::VectorXd weight;
    Eigen::VectorXd desired;
    Eigen::VectorXd currentState;
    std::string type;
    std::string name;
    bool isActive;

    friend std::ostream& operator<<(std::ostream &out, const TaskParameters& params)
        {
            out << "kp = " << params.kp << std::endl;
            out << "kd = " << params.kd << std::endl;
            out << "dimension = " << params.dimension << std::endl;
            out << "weight = " << params.weight.transpose() << std::endl;
            out << "desired = " << params.desired.transpose() << std::endl;
            out << "currentState = " << params.currentState.transpose() << std::endl;
            out << "type = " << params.type << std::endl;
            out << "name = " << params.name << std::endl;
            out << "isActive = " << params.isActive << std::endl;
            return out;
        }
};



class ControlThread: public yarp::os::RateThread
{

public:
    // Constructor
    ControlThread(int period, const std::string& taskRpcPortName);
    ~ControlThread();

    static int threadId;


    // RateThread virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    // ControlThread pure virtual functions
    virtual bool ct_threadInit()=0;
    virtual void ct_threadRelease()=0;
    virtual void ct_run()=0;

    // ControlThread functions
    std::string getThreadType(){return controlThreadType;}
    bool deactivateTask();
    bool activateTask();




    /************** controlInputCallback *************/
    class inputCallback : public yarp::os::PortReader {
        private:
            ControlThread& ctBase;

        public:
            inputCallback(ControlThread& ctBaseRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
    };
    /************** controlInputCallback *************/

protected:

    void setThreadType(const std::string& _threadType = "ControlThread"){controlThreadType = _threadType;}

    std::string controlThreadType;

    // Yarp control ports
    std::string inputPortName, outputPortName;
    yarp::os::Port inputPort, outputPort;

    //Yarp RPC client
    std::string taskRpcServerName, threadRpcClientName;
    yarp::os::RpcClient threadRpcClient;

    //Yarp network
    yarp::os::Network yarp;

    bool openControlPorts();
    bool connectControlPorts();

    bool isFirstInputBottle;
    inputCallback* inpCallback;
    Eigen::VectorXd currentStateVector;
    bool parseInput(yarp::os::Bottle* input);

    Eigen::VectorXd getCurrentState();
    bool waitingForFirstStateMessage;
    void sendGetStateMessage();

    double controlThreadPeriod;

    TaskParameters originalTaskParams;
    TaskParameters currentTaskParams;

    int weightDimension;
    int stateDimension;

    bool getTaskDimensions();
    bool getTaskParameters(TaskParameters& TP);


    double closePortTimeout;
};

#endif
