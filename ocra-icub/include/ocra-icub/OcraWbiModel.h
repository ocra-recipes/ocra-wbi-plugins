/*! \file       OcraWbiModel.h
 *  \brief      Implementation of ocra::Model using wholeBodyInterface.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-icub.
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

#ifndef OCRA_WBI_MODEL_H
#define OCRA_WBI_MODEL_H

#include <memory>

#include "ocra/control/Model.h"
#include <wbi/wbi.h>
#include <yarp/os/Log.h>
#include "ocra-icub/OcraWbiConversions.h"
#include "ocra-icub/Utilities.h"

namespace ocra_icub
{

class OcraWbiModel: public ocra::Model
{
// CLASS_POINTER_TYPEDEFS(OcraWbiModel)

public:



//===========================Constructor/Destructor===========================//
    OcraWbiModel(const std::string& robotName, const int robotNumDOF, std::shared_ptr<wbi::wholeBodyInterface> wbi, const bool freeRoot);
    virtual ~OcraWbiModel();

//=============================General functions==============================//
    virtual int                          nbSegments               () const;
    virtual const Eigen::VectorXd&       getActuatedDofs          () const;
    virtual const Eigen::VectorXd&       getJointLowerLimits      () const;
    virtual const Eigen::VectorXd&       getJointUpperLimits      () const;
    virtual const Eigen::VectorXd&       getJointPositions        () const;
    virtual const Eigen::VectorXd&       getJointVelocities       () const;
    virtual const Eigen::VectorXd&       getJointTorques          () const;

    virtual const Eigen::Displacementd&  getFreeFlyerPosition     () const;
    virtual const Eigen::Twistd&         getFreeFlyerVelocity     () const;

    virtual const std::string&           getJointName             (int index) const;
    virtual const int                    getSegmentIndex          (std::string segmentName) const;

//=============================Dynamic functions==============================//
    virtual const Eigen::MatrixXd&       getInertiaMatrix         () const;
    virtual const Eigen::MatrixXd&       getInertiaMatrixInverse  () const;
    virtual const Eigen::MatrixXd&       getDampingMatrix         () const;
    virtual const Eigen::VectorXd&       getNonLinearTerms        () const;
    virtual const Eigen::VectorXd&       getLinearTerms           () const;
    virtual const Eigen::VectorXd&       getGravityTerms          () const;

//===============================CoM functions================================//
    virtual double                                         getMass            () const;
    virtual const Eigen::Vector3d&                         getCoMPosition     () const;
    virtual const Eigen::Vector3d&                         getCoMVelocity     () const;
    virtual const Eigen::Vector3d&                         getCoMJdotQdot     () const;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobian     () const;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobianDot  () const;
    virtual const Eigen::Vector3d&                         getCoMAngularVelocity     () const;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMAngularJacobian     () const;

//=============================Segment functions==============================//
    virtual const Eigen::Displacementd&                    getSegmentPosition          (int index) const;
    virtual const Eigen::Twistd&                           getSegmentVelocity          (int index) const;
    virtual double                                         getSegmentMass              (int index) const;
    virtual const Eigen::Vector3d&                         getSegmentCoM               (int index) const;
    virtual const Eigen::Matrix<double,6,6>&               getSegmentMassMatrix        (int index) const;
    virtual const Eigen::Vector3d&                         getSegmentMomentsOfInertia  (int index) const;
    virtual const Eigen::Rotation3d&                       getSegmentInertiaAxes       (int index) const;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJacobian          (int index) const;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJdot              (int index) const;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getJointJacobian            (int index) const;
    virtual const Eigen::Twistd&                           getSegmentJdotQdot          (int index) const;

    void printAllData();

    // void getJointTorques(Eigen::VectorXd& wbiTorques);




protected:

//===========================Update state functions===========================//
    virtual void  doSetState(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot);

    virtual void  doSetState(const Eigen::Displacementd& H_root, const Eigen::VectorXd& q, const Eigen::Twistd& T_root, const Eigen::VectorXd& q_dot);

    virtual void                doSetJointPositions     (const Eigen::VectorXd& q);
    virtual void                doSetJointVelocities    (const Eigen::VectorXd& dq);
    virtual void                doSetFreeFlyerPosition  (const Eigen::Displacementd& Hroot);
    virtual void                doSetFreeFlyerVelocity  (const Eigen::Twistd& Troot);

//============================Index name functions============================//
    virtual int                 doGetSegmentIndex       (const std::string& name) const;
    virtual const std::string&  doGetSegmentName        (int index) const;
    virtual int                 doGetDofIndex           (const std::string& name) const;
    virtual const std::string&  doGetDofName            (int index) const;
    virtual const std::string   doSegmentName           (const std::string& name) const;
    virtual const std::string   doDofName               (const std::string& name) const;

private:
    std::shared_ptr<wbi::wholeBodyInterface> robot; // Access to wholeBodyInterface
    struct OcraWbiModel_pimpl;
    boost::shared_ptr<OcraWbiModel_pimpl> owm_pimpl; // where all internal data are saved
};
} /* ocra_icub */

#endif // OCRA_WBI_MODEL_H
