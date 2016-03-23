#include <ocra-icub/IcubControllerClient.h>

using namespace ocra_icub;

IcubControllerClient::IcubControllerClient()
{

}


IcubControllerClient::IcubControllerClient(std::shared_ptr<ocra_icub::OcraWbiModel> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{

}

IcubControllerClient::~IcubControllerClient()
{

}

void IcubControllerClient::printHelp()
{
    std::cout << "This is a help statement from the client." << std::endl;
}
