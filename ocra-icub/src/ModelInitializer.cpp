#include <ocra-icub/ModelInitializer.h>

using namespace ocra_icub;

int ModelInitializer::MODEL_INITIALIZER_COUNT = 0;

ModelInitializer::ModelInitializer()
{
    modInitNumber = ++MODEL_INITIALIZER_COUNT;
    if( getConfigurationInfoFromControllerServer() )
    {
        if ( configureWbi() ){
             constructModel();
        }
    }
}

ModelInitializer::~ModelInitializer()
{
    /* Stop the WBI threads. */
    if(robotInterface){
        if(!robotInterface->close())
            yLog.error() << "Error while closing robot interface";
    }
}

bool ModelInitializer::getConfigurationInfoFromControllerServer()
{
    bool retVal = false;
    yarp::os::RpcClient clientPort;
    std::string portName = "/tmpClientPort";
    clientPort.open(portName.c_str());
    yarp::os::Network yarp;
    double network_timeout = 10.0;
    if (!yarp.checkNetwork(network_timeout))
    {
        yLog.fatal() << "YARP network is not available";
        return false;
    }

    yarp.connect(portName.c_str(), "/ocra-icub-server/info/rpc:i");

    yarp::os::Bottle message, reply;
    message.addInt(GET_MODEL_CONFIG_INFO);
    clientPort.write(message, reply);
    if(reply.size()!=0)
    {
        wbiConfigFilePath = reply.get(0).asString();
        robotName = reply.get(1).asString();
        isFloatingBase = reply.get(2).asInt();
        retVal = true;
    }
    else
    {
        yLog.fatal() << "Controller server not responding. Is it running?";
    }


    yLog.info() << "Found WBI config file here: " << wbiConfigFilePath;
    yLog.info() << "Robot name is: " << robotName;
    yLog.info() << "Robot has floating base: " << isFloatingBase;

    return retVal;
}


bool ModelInitializer::configureWbi()
{


    // Create the yarpWBI options
    yarp::os::Property yarpWbiOptions;
    // Parse from the file we found.
    yarpWbiOptions.fromConfigFile(wbiConfigFilePath);
    // Overwrite the robot parameter that could be present in wbi_conf_file
    yarpWbiOptions.put("robot", robotName);
    // Create the wholeBodyInterface.
    robotInterface = std::make_shared<yarpWbi::yarpWholeBodyInterface>(getUniqueWbiName().c_str(), yarpWbiOptions);

    if (!robotInterface) {
        yLog.fatal() << "Could not create the yarp wholeBodyInterface!";
        return false;
    }
    // Add the robot's specific joints to the WBI.
    wbi::IDList robotJoints;
    std::string robotJointsListName = "ROBOT_MAIN_JOINTS";
    if(!yarpWbi::loadIdListFromConfig(robotJointsListName, yarpWbiOptions, robotJoints))
    {
        yLog.error() << "Impossible to load wbiId joint list with name: " << robotJointsListName;
        return false;
    }
    robotInterface->addJoints(robotJoints);

    // Make sure all the add* functions are done before the "init"
    if(!robotInterface->init())
    {
        yLog.fatal() << "Error while initializing whole body interface. Closing.";
        return false;
    }

    return true;
}

void ModelInitializer::constructModel()
{
    model = std::make_shared<OcraWbiModel>(robotName, robotInterface->getDoFs(), robotInterface, isFloatingBase);
}

std::string ModelInitializer::getUniqueWbiName()
{
    std::string robotInterfaceName = "ModelInitializer_"+ std::to_string(modInitNumber) +"_WBI/";

    return robotInterfaceName;
}
