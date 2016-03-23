#ifndef ICUB_CONTROLLER_CLIENT
#define ICUB_CONTROLLER_CLIENT

#include <ocra-recipes/ControllerClient.h>
#include <ocra-icub/OcraWbiModel.h>
#include <ocra-icub/Utilities.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>


namespace ocra_icub
{
class IcubControllerClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(IcubControllerClient)

public:
    IcubControllerClient ();
    IcubControllerClient (std::shared_ptr<ocra_icub::OcraWbiModel> modelPtr, const int loopPeriod=10);
    virtual ~IcubControllerClient ();

    virtual void printHelp();

    /*! Configures the module by parsing the RF contents.
     *  \param rf A resource finder instance which is initialized from the command line args.
     *
     *  \return True or false if the configuration was successful.
     */
    virtual bool configure(yarp::os::ResourceFinder &rf){return true;};

protected:
    virtual bool initialize(){return true;}
    virtual void release(){/* Do nothing. */}
    virtual void loop() = 0;

private:


};

} /* ocra_icub */
#endif
