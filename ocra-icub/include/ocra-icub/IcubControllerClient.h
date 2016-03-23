#ifndef ICUB_CONTROLLER_CLIENT
#define ICUB_CONTROLLER_CLIENT

#include <ocra-recipes/ControllerClient.h>
#include <ocra-icub/OcraWbiModel.h>
#include <ocra-icub/Utilities.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>


namespace ocra_icub
{
class IcubControllerClient : public ocra_recipes::ControllerClient
{

public:
    IcubControllerClient ();
    IcubControllerClient (std::shared_ptr<ocra_icub::OcraWbiModel> modelPtr, const int loopPeriod=10);
    virtual ~IcubControllerClient ();

protected:
    virtual bool initialize(){return true;}
    virtual void release(){/* Do nothing. */}
    virtual void loop() = 0;

private:

};

} /* ocra_icub */
#endif
