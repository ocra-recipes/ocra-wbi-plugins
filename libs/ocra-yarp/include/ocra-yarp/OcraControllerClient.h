/*! \file       OcraControllerClient.h
 *  \brief      A base class for all controller clients.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-yarp.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OCRA_CONTROLLER_CLIENT_H
#define OCRA_CONTROLLER_CLIENT_H

#include <iostream>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include "ocra-yarp/OcraYarpTools.h"

namespace ocra_yarp
{

class OcraControllerClient: public yarp::os::RateThread
{
DEFINE_CLASS_POINTER_TYPEDEFS(OcraControllerClient)

    // RateThread virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    // OcraControllerClient pure virtual functions
    virtual bool client_threadInit()=0;
    virtual void client_threadRelease()=0;
    virtual void client_run()=0;
};

} // namespace ocra_yarp

#endif // OCRA_CONTROLLER_CLIENT_H
