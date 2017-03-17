#include "sitting-demo/SittingDemoClient.h"
SittingDemoClient::SittingDemoClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    xDisp = 0.0;
    yDisp = 0.0;
    zDisp = 0.0;
}

SittingDemoClient::~SittingDemoClient()
{
    // add your code here...
    if(comTrajThread){
        comTrajThread->stop();
    }
}

bool SittingDemoClient::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("taskName")) {
        taskName = rf.find("taskName").asString();
    } else {
        taskName = "ComTask";
    }
    if (rf.check("y")) {
        yDisp = rf.find("y").asDouble();
    }
    if (rf.check("z")) {
        zDisp = rf.find("z").asDouble();
    }
}

bool SittingDemoClient::initialize()
{
    yarp::os::Time::delay(2.0);
    comTask = std::make_shared<ocra_recipes::TaskConnection>(taskName);

    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;

    currentDesiredPosition = comTask->getDesiredTaskState().getPosition().getTranslation();

    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::NONE;

    comTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, taskName, currentDesiredPosition, trajType, termStrategy);

    comTrajThread->setMaxVelocity(0.01);
    comTrajThread->start();

    return true;
}

void SittingDemoClient::release()
{
    // add your code here...
}

void SittingDemoClient::loop()
{
    yarp::os::Time::delay(2.0);

    Eigen::Vector3d comPos = comTask->getTaskState().getPosition().getTranslation();
    Kp = comTask->getStiffnessMatrix();
    Kd = comTask->getDampingMatrix();
    std::cout << "\n=============================================" << std::endl;
    std::cout << taskName << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "Kp = \n" << Kp << std::endl;
    std::cout << "Kd = \n" << Kd << std::endl;
    std::cout << "CoM Position:" << std::endl;
    std::cout << "x = " << comPos(0) << "   y = " << comPos(1) << "   z = " << comPos(2) << std::endl;
    std::cout << "Desired CoM Position:" << std::endl;
    std::cout << "x = " << currentDesiredPosition(0) << "   y = " << currentDesiredPosition(1) << "   z = " << currentDesiredPosition(2) << std::endl;

    std::string key;
    double value;
    bool goodKey = true;

    do {
        std::cout << "\nType 'x', 'y', 'z', 'kp', or 'kd' to change value or 'exit' to quit:" << std::endl;
        std::cin >> key;
        goodKey =   (key == "x")    ||
                    (key == "y")    ||
                    (key == "z")    ||
                    (key == "kp")   ||
                    (key == "kpz")   ||
                    (key == "kd")   ||
                    (key == "kdz")   ||
                    (key == "exit");
        if (!goodKey) {
            std::cout << "invalid key..." << std::endl;
        }
    } while( !goodKey );

    if (key == "exit") {
        std::cout << "Exiting..." << std::endl;
        stop();
    } else {
        std::cout << "Enter value: " << std::endl;
        std::cin >> value;

        xDisp = 0.0;
        yDisp = 0.0;
        zDisp = 0.0;

        if ( key == "x") {
            xDisp = value / 100.0; // cm to m
            moveCom();
        } else if ( key == "y") {
            yDisp = value / 100.0; // cm to m
            moveCom();
        } else if ( key == "z") {
            zDisp = value / 100.0; // cm to m
            moveCom();
        } else if ( key == "kp") {
            comTask->setStiffness(value);
        } else if ( key == "kd") {
            comTask->setDamping(value);
        } else if ( key == "kpz") {
            Eigen::MatrixXd kpMat = comTask->getStiffnessMatrix();
            kpMat(2,2) = value;
            comTask->setStiffness(kpMat);
        } else if ( key == "kdz") {
            Eigen::MatrixXd kdMat = comTask->getDampingMatrix();
            kdMat(2,2) = value;
            comTask->setDamping(kdMat);
        } else {
            std::cout << "Error BAD KEY" << std::endl;
        }
    }
}


void SittingDemoClient::moveCom()
{
    currentDesiredPosition = comTask->getDesiredTaskState().getPosition().getTranslation();

    currentDesiredPosition(0) += xDisp;
    currentDesiredPosition(1) += yDisp;
    currentDesiredPosition(2) += zDisp;

    comTrajThread->setTrajectoryWaypoints(currentDesiredPosition);
}
