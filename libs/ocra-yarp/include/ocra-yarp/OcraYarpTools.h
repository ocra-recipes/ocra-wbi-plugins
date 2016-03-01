/*! \file       OcraYarpTools.h
 *  \brief      Some useful tools for the ocra-yarp lib.
 *  \details    This file is for all the various macros and helper classes that are generic to ocra-yarp. It is also a place for all the repeated includes like standard stl includes etc.
 *  \author     [Ryan Lober](http://www.ryanlober.com)
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

#ifndef OCRA_YARP_TOOLS_H
#define OCRA_YARP_TOOLS_H

/*
 *  Let's include some shit!!!! Here we go...
 */

// STL includes
#include <memory>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>


// Yarp includes
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

// WholeBodyInterface includes
#include <wbi/wbi.h>

// Ocra includes
#include "ocra/control/Model.h"

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Lgsm>



namespace ocra_yarp
{
/*! Macro function which basically just defines pointer typdefs for classes. Note that normally macros are evil monsters but I think this is a perfect usage case to cut down on repeated code.
 *  \param Class Just pop this bad boy below the class definition and it will do the rest.
 */
#ifndef DEFINE_CLASS_POINTER_TYPEDEFS
#define DEFINE_CLASS_POINTER_TYPEDEFS(Class) public:\
using ptr           = std::shared_ptr   <Class>;  \
using shared_ptr    = std::shared_ptr   <Class>;  \
using unique_ptr    = std::unique_ptr   <Class>;  \
using weak_ptr      = std::weak_ptr     <Class>;  \
using const_ptr           = const std::shared_ptr   <Class>;  \
using const_shared_ptr    = const std::shared_ptr   <Class>;  \
using const_unique_ptr    = const std::unique_ptr   <Class>;  \
using const_weak_ptr      = const std::weak_ptr     <Class>;
#endif



static constexpr double DEG_TO_RAD = M_PI/180.0;


void getNominalPosture(ocra::Model &model, Eigen::VectorXd &q);
void getHomePosture(ocra::Model &model, Eigen::VectorXd &q);

} // namespace ocra_yarp
#endif //OCRA_YARP_TOOLS_H
