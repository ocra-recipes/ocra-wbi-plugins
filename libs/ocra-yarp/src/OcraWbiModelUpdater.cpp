/*! \file       OcraWbiModelUpdater.cpp
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

#include "ocra-yarp/OcraWbiModelUpdater.h"

using namespace ocra_yarp;

OcraWbiModelUpdater::OcraWbiModelUpdater()
{
}

OcraWbiModelUpdater::~OcraWbiModelUpdater()
{
}

bool OcraWbiModelUpdater::initialize(wbi::wholeBodyInterface* wbiPointer, ocra::Model* modelPointer)
{
    fb_qRad = Eigen::VectorXd::Zero(wbiPointer->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(wbiPointer->getDoFs());

    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(6);

    fb_Hroot_Vector = yarp::sig::Vector(16, 0.0);
    fb_Troot_Vector = yarp::sig::Vector(6, 0.0);

    return update(wbiPointer, modelPointer);
}

bool OcraWbiModelUpdater::update(wbi::wholeBodyInterface* wbiPointer, ocra::Model* modelPointer)
{
    if (wbiPointer==NULL || modelPointer == NULL) {
        return false;
    }
    else {

        bool gotEstimatesFromWbi = true;

        gotEstimatesFromWbi &= wbiPointer->getEstimates(wbi::ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
        gotEstimatesFromWbi &= wbiPointer->getEstimates(wbi::ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);

        if (!modelPointer->hasFixedRoot())
        {
            // Get root position as a 12x1 vector and get root vel as a 6x1 vector
            gotEstimatesFromWbi &= wbiPointer->getEstimates(wbi::ESTIMATE_BASE_POS, fb_Hroot_Vector.data());
            gotEstimatesFromWbi &= wbiPointer->getEstimates(wbi::ESTIMATE_BASE_VEL, fb_Troot_Vector.data());
            // Convert to a wbi::Frame and a "fake" Twistd
            wbi::frameFromSerialization(fb_Hroot_Vector.data(), fb_Hroot);
            fb_Troot = Eigen::Twistd(fb_Troot_Vector[0], fb_Troot_Vector[1], fb_Troot_Vector[2], fb_Troot_Vector[3], fb_Troot_Vector[4], fb_Troot_Vector[5]);

            //TODO: Convert setwbistate() call to setState() to avoid the dynamic cast. This is used because Hroot_wbi is kept in the ocraWbiModel pimpl and used for the mass and nonlinear matrix calcs.
            dynamic_cast<OcraWbiModel*>(modelPointer)->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
        }
        else
        {
            modelPointer->setState(fb_qRad, fb_qdRad);
        }
        return gotEstimatesFromWbi;
    }
}
