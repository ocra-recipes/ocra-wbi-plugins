/*! \file       OcraWbiConversions.h
 *  \brief      A utility class full of static conversion functions.
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

#ifndef OCRA_WBI_CONVERSIONS_H
#define OCRA_WBI_CONVERSIONS_H

#include <iostream>
#include <map>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/Log.h>
#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include "ocra-yarp/OcraYarpTools.h"


namespace ocra_yarp{

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRm;

/* Conversions between types/conventions used in Eigen and WBI */
class OcraWbiConversions
{
DEFINE_CLASS_POINTER_TYPEDEFS(OcraWbiConversions)
public:
    static bool eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame);
    static bool wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp);
    static bool wbiToOcraTwistVector(Eigen::Twistd &t_wbi, Eigen::Twistd &t_ocra);
    static bool wbiToOcraSegJacobian(const Eigen::MatrixXd &jac, Eigen::MatrixXd &J);
    static bool wbiToOcraCoMJacobian(const Eigen::MatrixXd &jac, Eigen::Matrix<double,3,Eigen::Dynamic> &J);
    static bool eigenRowMajorToColMajor(const MatrixXdRm &M_rm, Eigen::MatrixXd &M);
    static bool wbiToOcraMassMatrix(int qdof, const Eigen::MatrixXd &M_wbi, Eigen::MatrixXd &M_ocra);
    static bool wbiToOcraBodyVector(int qdof, const Eigen::VectorXd &v_wbi, Eigen::VectorXd &v_ocra);
    static bool eigenToYarpVector(const Eigen::VectorXd &eigenVector, yarp::sig::Vector &yarpVector);

    static const int DIM_T = 3;
    static const int DIM_R = 3;
};
} // namespace ocra_yarp
#endif //OCRA_WBI_CONVERSIONS_H
