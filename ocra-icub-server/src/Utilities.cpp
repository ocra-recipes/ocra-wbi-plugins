/*! \file       Utilities.cpp
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
#include "ocra-icub-server/Utilities.h"


namespace ocra_yarp
{

void getNominalPosture(const ocra::Model& model, Eigen::VectorXd &q)
{
    q[model.getDofIndex("torso_pitch")] = M_PI / 18;
    q[model.getDofIndex("r_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("r_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("l_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("r_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("l_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("r_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("l_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("r_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("l_knee")] = -M_PI / 6;
    q[model.getDofIndex("r_knee")] = -M_PI / 6;
}

void getHomePosture(const ocra::Model& model, Eigen::VectorXd &q)
{
    q[model.getDofIndex("l_shoulder_roll")]=   20.0*DEG_TO_RAD;//PI/8.0;
    q[model.getDofIndex("r_shoulder_roll")]=   20.0*DEG_TO_RAD;//PI/8.0;
    q[model.getDofIndex("l_shoulder_pitch")]=  -25.0*DEG_TO_RAD;//-PI/8.0;
    q[model.getDofIndex("r_shoulder_pitch")]=  -25.0*DEG_TO_RAD;//-PI/8.0;
    q[model.getDofIndex("l_elbow")]        =   50.0*DEG_TO_RAD;//PI/4.0;
    q[model.getDofIndex("r_elbow")]        =   50.0*DEG_TO_RAD;//PI/4.0;
}
}
