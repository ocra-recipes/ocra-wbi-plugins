#include "walking-client/WalkingClient.h"
WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    // add your code here...
}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

bool WalkingClient::initialize()
{
    // Create ZMPPreviewController object
    
    // add your code here...
    return true;
}

void WalkingClient::release()
{
    // add your code here...
}

void WalkingClient::loop()
{
    // add your code here...
}
