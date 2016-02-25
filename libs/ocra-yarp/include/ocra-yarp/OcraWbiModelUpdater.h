/*! \file       OcraWbiModelUpdater.h
 *  \brief      A function for updating a OcraWbiModel by calling the appropriate WBI estimators.
 *  \details    This was created to avoid repeating this code over and over in routines which use the OcraWbiModel for state stuff.
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \todo       Convert setwbistate() call to setState() to avoid the dynamic cast. This is used because Hroot_wbi is kept in the ocraWbiModel pimpl and used for the mass and nonlinear matrix calcs.
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

#ifndef OCRA_WBI_MODEL_UPDATER_H
#define OCRA_WBI_MODEL_UPDATER_H

#include <memory>
#include "ocra-yarp/OcraWbiModel.h"

namespace ocra_yarp
{
class OcraWbiModelUpdater
{
public:
    OcraWbiModelUpdater();
    ~OcraWbiModelUpdater();
    bool initialize(std::shared_ptr<wbi::wholeBodyInterface> wbiPointer, ocra::Model* modelPointer);
    bool update(std::shared_ptr<wbi::wholeBodyInterface> wbiPointer, ocra::Model* modelPointer);

private:
    Eigen::VectorXd fb_qRad; // vector that contains the encoders read from the wbiPointer
    Eigen::VectorXd fb_qdRad; // vector that contains the derivative of encoders read from the wbiPointer

    yarp::sig::Vector fb_Hroot_Vector;
    yarp::sig::Vector fb_Troot_Vector;

    wbi::Frame fb_Hroot; // vector that position of root
    Eigen::Twistd fb_Troot; // vector that contains the twist of root

    static const int ALL_JOINTS = -1;
};
} // namespace ocra_yarp
#endif //CONTROL_THREAD_H
