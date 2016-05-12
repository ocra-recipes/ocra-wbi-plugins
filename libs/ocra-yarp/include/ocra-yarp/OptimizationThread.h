/*! \file       OptimizationThread.h
 *  \brief      A generic thread for launching optimization routines (typically slow ones).
 *  \warning    For the moment this does nothing
 *  \todo       Implement this class.
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

#ifndef OPTIMIZATION_THREAD_H
#define OPTIMIZATION_THREAD_H


#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include "ocra-yarp/OcraYarpTools.h"

namespace ocra_yarp
{
class OptimizationThread : yarp::os::RateThread
{
DEFINE_CLASS_POINTER_TYPEDEFS(OptimizationThread)

public:

protected:
    yarp::os::Network yarp;

private:


};
} // namespace ocra_yarp
#endif // OPTIMIZATION_THREAD_H
