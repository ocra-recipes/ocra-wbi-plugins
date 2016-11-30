#include "walking-client/WalkingClient.h"
WalkingClient::WalkingClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod),
_zmpParams(std::make_shared<ZmpControllerParams>(1, model->getMass(), model->getCoMPosition().operator()(2), 9.8, 0.05) ),
_zmpController(std::make_shared<ZmpController>(loopPeriod, modelPtr, _zmpParams))
{
    //TODO: These parameters should be read from the configuration file of the client.
//    _zmpParams = ZmpControllerParams(1, model->getMass(), model->getCoMPosition().operator()(2), 9.8, 0.05);
//    _zmpParams.kf = 1;
//    _zmpParams.m = model->getMass();
//    _zmpParams.cz = model->getCoMPosition().operator()(2);
//    _zmpParams.g = 9.8;
//    _zmpParams.d = 0.05;
//    this->_zmpControllerObj = std::make_shared<ZmpController>(loopPeriod, modelPtr, );
    // add your code here...
}

WalkingClient::~WalkingClient()
{
    // add your code here...
}

bool WalkingClient::initialize()
{
    // Create ZMPController object
    
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
