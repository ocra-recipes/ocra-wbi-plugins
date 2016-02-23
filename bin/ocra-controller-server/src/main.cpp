#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>

#include <ocra-yarp/OcraControllerServerModule.h>

#define DEFAULT_YARP_CONTEXT "ocra-controller-server"



int main (int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    ocra_yarp::OcraControllerServerModule module;
    yarp::os::Network yarp;
    yarp::os::Log yLog;


    rf.setVerbose(true);
    rf.setDefaultConfigFile("ocra-controller-server.ini"); //default config file name.
    rf.setDefaultContext(DEFAULT_YARP_CONTEXT); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {

        module.printHelp();

        return 0;
    }


    double network_timeout = 10.0;
    if (!yarp.checkNetwork(network_timeout))
    {
        yLog.fatal() << "YARP network is not available";
        return -1;
    }


    return module.runModule(rf);
}
