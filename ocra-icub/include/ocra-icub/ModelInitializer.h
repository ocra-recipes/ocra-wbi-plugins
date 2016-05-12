#ifndef MODEL_INITIALIZER_H
#define MODEL_INITIALIZER_H

#include <ocra-icub/OcraWbiModel.h>
#include <ocra/control/Model.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <yarp/os/RpcClient.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

namespace ocra_icub
{

class ModelInitializer {
private:
    std::shared_ptr<yarpWbi::yarpWholeBodyInterface> robotInterface;
    std::shared_ptr<ocra::Model> model;

    std::string wbiConfigFilePath;
    std::string robotName;
    bool isFloatingBase;

    yarp::os::Log yLog;

    bool configureWbi();
    void constructModel();
    bool getConfigurationInfoFromControllerServer();

    std::string getUniqueWbiName();
    int modInitNumber;
    static int MODEL_INITIALIZER_COUNT;

public:
    ModelInitializer ();
    virtual ~ModelInitializer ();

    std::shared_ptr<ocra::Model> getModel(){return model;}


};

} /* ocra_icub */

#endif //MODEL_INITIALIZER_H
