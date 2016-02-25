#ifndef OCRA_YARP_VOCAB_H
#define OCRA_YARP_VOCAB_H

#include <iostream>

namespace ocra_yarp
{

enum OCRA_YARP_MESSAGE_TAG
{
    TEST
};

enum OCRA_CONTROLLER_TYPE
{
    WOCRA_CONTROLLER = 10,
    GOCRA_CONTROLLER = 11,
    HOCRA_CONTROLLER = 12
};

} // namespace ocra_yarp
#endif // OCRA_YARP_VOCAB_H
