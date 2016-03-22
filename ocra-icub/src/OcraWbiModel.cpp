/*! \file       OcraWbiModel.cpp
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

#include <ocra-icub/OcraWbiModel.h>


#define ALL_JOINTS -1
#define FREE_ROOT_DOF 6
#define COM_POS_DIM 3
#define TRANS_ROT_DIM 6

#define GRAVITY_CONSTANT -9.81

typedef  Eigen::Displacementd::AdjointMatrix  AdjointMatrix;

const double g_vector[3] = {0, 0, GRAVITY_CONSTANT};




struct OcraWbiModel::OcraWbiModel_pimpl
{

public:
    bool                                                    freeRoot;
    int                                                     nbDofs;
    int                                                     nbInternalDofs; // nbDofs + FREE_ROOT_DOF if free root, otherwise the same as nbDofs

    int                                                     nbSegments; // nbInternalDofs+1?
    Eigen::VectorXd                                         actuatedDofs; // which joints are actuated
    Eigen::VectorXd                                         lowerLimits; // lower q of joints
    Eigen::VectorXd                                         upperLimits; // upper q of joints
    Eigen::VectorXd                                         q; // state variable
    Eigen::VectorXd                                         dq; // derivative of q
    Eigen::VectorXd                                         tau; // joint torques
    Eigen::Displacementd                                    Hroot; // translation of root
    wbi::Frame                                              Hroot_wbi;
    Eigen::Twistd                                           Troot; // twist of root (velocity)
    Eigen::Twistd                                           Troot_wbi; // twist of root (velocity)
    Eigen::MatrixXd                                         M; // Mass inertia matrix (col major for ocra control)
    Eigen::MatrixXd                                         M_full; // Full Mass inertia matrix (col major)
    MatrixXdRm                                              M_full_rm; // Mass inertia matrix (from WholeBodyInterface, row major)
    Eigen::MatrixXd                                         Minv; // Inverse of mass inertia matrix (col major for ocra control)
    Eigen::MatrixXd                                         B; // Not set, set to ZERO for now (col major for ocra control)
    Eigen::VectorXd                                         nl; // non-linear terms in EOM (set as coriolis/centrifugal effects)
    Eigen::VectorXd                                         nl_full; // non-linear terms in EOM (full vector from WBI)
    Eigen::VectorXd                                         l; // linear terms in EOM (set this to be zero)
    Eigen::VectorXd                                         g; // gravity term in EOM
    Eigen::VectorXd                                         g_full; // gravity term in EOM (full vector from WBI)
    double                                                  total_mass;
    Eigen::Vector3d                                         pos_com; // COM position
    Eigen::Vector3d                                         vel_com; // COM linear velocity
    Eigen::Vector3d                                         vel_com_angular; // COM angular velocity
    Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>        J_com; // Jacobian matrix (col major for ocra control)
    Eigen::MatrixXd                                         J_com_full; // Jacobian matrix (full from WBI control)
    MatrixXdRm                                              J_com_rm; // Jacobian matrix (row major for WBI)
    Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>        J_com_angular; // Jacobian matrix (col major for ocra control)
    Eigen::MatrixXd                                         J_com_full_angular; // Jacobian matrix (col major MatrixXd for ocra control)
    MatrixXdRm                                              J_com_rm_angular; // Jacobian matrix (row major for WBI)
    Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>        DJ_com; // derivative of J
    MatrixXdRm                                              DJ_com_rm; // derivative of J
    Eigen::Vector3d                                         DJDq;

    Eigen::Displacementd                                    H_com;
    std::vector< Eigen::Displacementd >                     segPosition;
    std::vector< Eigen::Twistd >                            segVelocity;
    std::vector< double >                                   segMass; // not set
    std::vector< Eigen::Vector3d >                          segCoM; // not set
    std::vector< Eigen::Matrix<double,TRANS_ROT_DIM,TRANS_ROT_DIM> > segMassMatrix; // not set
/*
    std::vector< Eigen::Matrixd >                           segMassMatrix_cm;
    std::vector< MatrixXdRm >                               segMassMatrix_rm;
*/
    std::vector< Eigen::Vector3d >                          segMomentsOfInertia; // not set
    std::vector< Eigen::Rotation3d >                        segInertiaAxes; // not set

    std::vector< Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic> >   segJacobian;
//    std::vector < Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic> >   segJacobian_full;
    std::vector< Eigen::MatrixXd >                                      segJacobian_full;
    std::vector< Eigen::MatrixXd >                                      segJacobian_full_ocra;
    std::vector< MatrixXdRm >                                           segJacobian_rm;
    std::vector< Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic> >   segJdot; // not set
    std::vector< Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic> >   segJointJacobian;
    std::vector< Eigen::Twistd >                            segJdotQdot;
    std::map< std::string, int >                            segIndexFromName;
    std::vector< std::string >                              segNameFromIndex;
    Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>      Jroot;
    Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>      dJroot;

    OcraWbiModel_pimpl(int nbSeg, int ndof, int nDofFree)
        :nbSegments(nbSeg)
        ,q(Eigen::VectorXd::Zero(nDofFree-TRANS_ROT_DIM))
        ,dq(Eigen::VectorXd::Zero(nDofFree-TRANS_ROT_DIM))
        ,tau(Eigen::VectorXd::Zero(nDofFree-TRANS_ROT_DIM))
        ,Hroot(Eigen::Displacementd(0,0,1))
        ,Troot(Eigen::Twistd(0,0,0,0,0,0))
        ,Hroot_wbi(wbi::Frame())
        ,Troot_wbi(Eigen::Twistd(0,0,0,0,0,0))
        ,M(Eigen::MatrixXd::Zero(ndof, ndof))
        ,M_full(Eigen::MatrixXd::Zero(nDofFree, nDofFree))
        ,M_full_rm(Eigen::MatrixXd::Zero(nDofFree, nDofFree))
        ,B(Eigen::MatrixXd::Zero(ndof, ndof))
        ,nl(Eigen::VectorXd::Zero(ndof))
        ,nl_full(Eigen::VectorXd::Zero(nDofFree))
        ,l(Eigen::VectorXd::Zero(ndof))
        ,g(Eigen::VectorXd::Zero(ndof))
        ,g_full(Eigen::VectorXd::Zero(nDofFree))
        ,J_com(COM_POS_DIM, ndof)
        ,J_com_rm(TRANS_ROT_DIM, nDofFree)
        ,J_com_full(TRANS_ROT_DIM, nDofFree)
        ,J_com_angular(COM_POS_DIM, ndof)
        ,J_com_rm_angular(TRANS_ROT_DIM, nDofFree)
        ,J_com_full_angular(TRANS_ROT_DIM, nDofFree)
        ,DJ_com(Eigen::MatrixXd::Zero(COM_POS_DIM, ndof))
        ,DJ_com_rm(MatrixXdRm::Zero(COM_POS_DIM, nDofFree))
        ,DJDq(Eigen::Vector3d(0,0,0))
        ,segPosition(nbSeg, Eigen::Displacementd(0,0,0))
        ,segVelocity(nbSeg, Eigen::Twistd(0,0,0,0,0,0))
        ,segMass(nbSeg, 0)
        ,H_com(Eigen::Displacementd(0,0,0))
        ,segCoM(nbSeg, Eigen::Vector3d(0,0,0))
        ,segMassMatrix(nbSeg, Eigen::Matrix<double,6,6>::Zero())
        ,segMomentsOfInertia(nbSeg, Eigen::Vector3d(0,0,0))
        ,segInertiaAxes(nbSeg, Eigen::Rotation3d(1,0,0,0))
        ,segJacobian(nbSeg, Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>::Zero(TRANS_ROT_DIM,ndof))
        ,segJacobian_full(nbSeg, Eigen::MatrixXd::Zero(TRANS_ROT_DIM,nDofFree))
        ,segJacobian_full_ocra(nbSeg, Eigen::MatrixXd::Zero(TRANS_ROT_DIM,nDofFree))
        ,segJacobian_rm(nbSeg, MatrixXdRm::Zero(TRANS_ROT_DIM,nDofFree))
        ,segJdot(nbSeg, Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>::Zero(TRANS_ROT_DIM,ndof))
        ,segJointJacobian(nbSeg, Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>::Zero(TRANS_ROT_DIM,ndof))
        ,segJdotQdot(nbSeg, Eigen::Twistd(0,0,0,0,0,0))
    {

    }

};

//=================================  Class methods  =================================//
OcraWbiModel::OcraWbiModel(const std::string& robotName, const int robotNumDOF, std::shared_ptr<wholeBodyInterface> wbi, const bool freeRoot)
: ocra::Model(robotName, freeRoot?robotNumDOF+FREE_ROOT_DOF:robotNumDOF, freeRoot)
, robot(wbi)
, owm_pimpl(new OcraWbiModel_pimpl(robot->getFrameList().size(), freeRoot?robotNumDOF+FREE_ROOT_DOF:robotNumDOF, robotNumDOF+FREE_ROOT_DOF))
{
    owm_pimpl->freeRoot = freeRoot;
    int full_wbi_size = robotNumDOF+FREE_ROOT_DOF; // N+6

    // Initialise some constant variables

    // THIS GETS FROM WBI ROBOT
    // owm_pimpl->nbDofs = freeRoot?robot->getDoFs()+FREE_ROOT_DOF:robot->getDoFs();#include <wbiIcub/wholeBodyInterfaceIcub.h>
    // owm_pimpl->nbInternalDofs = robot->getDoFs();
    // THIS GETS FROM ocra MODEL
    owm_pimpl->nbDofs = nbDofs();
    owm_pimpl->nbInternalDofs = nbInternalDofs();

//    // Need to FIX THIS TO GET THE VALUE PROPERLY!
//    owm_pimpl->nbSegments = owm_pimpl->nbInternalDofs + 1;
    // Ones to indicate that all joints are actuated
    owm_pimpl->actuatedDofs = Eigen::VectorXd::Ones(owm_pimpl->nbInternalDofs);

    // Setup and get lower/upper joint limits
    owm_pimpl->lowerLimits = Eigen::VectorXd::Zero(owm_pimpl->nbInternalDofs);
    owm_pimpl->upperLimits = Eigen::VectorXd::Zero(owm_pimpl->nbInternalDofs);

    robot->getJointLimits(owm_pimpl->lowerLimits.data(), owm_pimpl->upperLimits.data(), ALL_JOINTS);

    // Get full M0
    MatrixXdRm M_rm_total_mass(full_wbi_size,full_wbi_size);
    robot->computeMassMatrix(owm_pimpl->q.data(), wbi::Frame(), M_rm_total_mass.data());
    owm_pimpl->total_mass = M_rm_total_mass(0,0);

}

OcraWbiModel::~OcraWbiModel()
{

}

int OcraWbiModel::nbSegments() const
{
    // set once, hence just return
    return owm_pimpl->nbSegments;
}

const Eigen::VectorXd& OcraWbiModel::getActuatedDofs() const
{
    // set once, hence just return
    return owm_pimpl->actuatedDofs;
}

const Eigen::VectorXd& OcraWbiModel::getJointLowerLimits() const
{
    // set once, hence just return
    return owm_pimpl->lowerLimits;
}

const Eigen::VectorXd& OcraWbiModel::getJointUpperLimits() const
{
    // set once, hence just return
    return owm_pimpl->upperLimits;
}

const Eigen::VectorXd& OcraWbiModel::getJointPositions() const
{
    // set by setState or setJointPositions
    return owm_pimpl->q;
}

const Eigen::VectorXd& OcraWbiModel::getJointVelocities() const
{
    // set by setState or setJointVelocities
    return owm_pimpl->dq;
}

const Eigen::VectorXd& OcraWbiModel::getJointTorques() const
{
    robot->getEstimates(ESTIMATE_JOINT_TORQUE, owm_pimpl->tau.data(), ALL_JOINTS);
    return owm_pimpl->tau;
}

const std::string& OcraWbiModel::getJointName(int index) const
{
    return doGetDofName(index);
}

const int OcraWbiModel::getSegmentIndex(std::string segmentName) const
{
    return doGetSegmentIndex(segmentName);
}



const Eigen::Displacementd& OcraWbiModel::getFreeFlyerPosition() const
{
    // set by setState or setFreeFlyerPosition
    return owm_pimpl->Hroot;
}

const Eigen::Twistd& OcraWbiModel::getFreeFlyerVelocity() const
{
    // set by setState or setFreeFlyerVelocity
    return owm_pimpl->Troot;
}

const Eigen::MatrixXd& OcraWbiModel::getInertiaMatrix() const
{
    bool res = robot->computeMassMatrix(owm_pimpl->q.data(), owm_pimpl->Hroot_wbi, owm_pimpl->M_full_rm.data());
    OcraWbiConversions::eigenRowMajorToColMajor(owm_pimpl->M_full_rm, owm_pimpl->M_full);

    if (owm_pimpl->freeRoot)
    {
        OcraWbiConversions::wbiToOcraMassMatrix(owm_pimpl->nbInternalDofs, owm_pimpl->M_full, owm_pimpl->M);
    }
    else
    {
        owm_pimpl->M = owm_pimpl->M_full.block(FREE_ROOT_DOF, FREE_ROOT_DOF, owm_pimpl->nbDofs, owm_pimpl->nbDofs);
    }

/*
    printf("Get Inertia Matrix\n");
    std::cout << owm_pimpl->M.lpNorm<Eigen::Infinity>() << std::endl;
*/

    return owm_pimpl->M;
}

const Eigen::MatrixXd& OcraWbiModel::getInertiaMatrixInverse() const
{
/*
    printf("Get Inertia Matrix Inverse\n");
*/
    getInertiaMatrix();
    owm_pimpl->Minv = owm_pimpl->M.inverse();
    return owm_pimpl->Minv;
}

const Eigen::MatrixXd& OcraWbiModel::getDampingMatrix() const
{
/*
    printf("Get Damping\n");
*/
    // not set so juts pass owm_pimpl->B back
    return owm_pimpl->B;
}

const Eigen::VectorXd& OcraWbiModel::getNonLinearTerms() const
{
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    bool res = robot->computeGeneralizedBiasForces(owm_pimpl->q.data(), owm_pimpl->Hroot_wbi, owm_pimpl->dq.data(), owm_pimpl->Troot_wbi.data(), zero.data(), owm_pimpl->nl_full.data());

    if (owm_pimpl->freeRoot)
        OcraWbiConversions::wbiToOcraBodyVector(owm_pimpl->nbInternalDofs, owm_pimpl->nl_full, owm_pimpl->nl);
    else
        owm_pimpl->nl = owm_pimpl->nl_full.segment(FREE_ROOT_DOF, owm_pimpl->nbDofs);

/*
    printf("Get Non Linear\n");
    std::cout << owm_pimpl->nl.transpose() << std::endl;
    printf("Get Non Linear Full\n");
    std::cout << owm_pimpl->nl_full.transpose() << std::endl;
*/

    return owm_pimpl->nl;
}

const Eigen::VectorXd& OcraWbiModel::getLinearTerms() const
{
/*
    printf("Get Linear\n");
*/

    return owm_pimpl->l;
}

const Eigen::VectorXd& OcraWbiModel::getGravityTerms() const
{
    Eigen::VectorXd dq_zero = Eigen::VectorXd::Zero(owm_pimpl->nbInternalDofs);
    Eigen::Vector3d g(g_vector);

    bool res = robot->computeGeneralizedBiasForces(owm_pimpl->q.data(), owm_pimpl->Hroot_wbi, dq_zero.data(), owm_pimpl->Troot_wbi.data(), g.data(), owm_pimpl->g_full.data());

    if (owm_pimpl->freeRoot)
        OcraWbiConversions::wbiToOcraBodyVector(owm_pimpl->nbInternalDofs, owm_pimpl->g_full, owm_pimpl->g);
    else
        owm_pimpl->g = owm_pimpl->g_full.segment(FREE_ROOT_DOF, owm_pimpl->nbDofs);

/*
    printf("Get Gravity\n");
    std::cout << owm_pimpl->g.transpose() << std::endl;
*/
    return owm_pimpl->g;
}

double OcraWbiModel::getMass() const
{
/*
    printf("Get Mass\n");
*/
    return owm_pimpl->total_mass;
}

const Eigen::Vector3d& OcraWbiModel::getCoMPosition() const
{
    Frame H;
    robot->computeH(owm_pimpl->q.data(),owm_pimpl->Hroot_wbi,wbi::iWholeBodyModel::COM_LINK_ID,H);

    OcraWbiConversions::wbiFrameToEigenDispd(H,owm_pimpl->H_com);
    owm_pimpl->pos_com = owm_pimpl->H_com.getTranslation();
/*
    printf("Get COM Poisiton\n");
    std::cout << owm_pimpl->pos_com << std::endl;
*/
    return owm_pimpl->pos_com;
}

const Eigen::Vector3d& OcraWbiModel::getCoMVelocity() const
{
/*
    printf("Get COM Velocity\n");
*/
    if (owm_pimpl->freeRoot)
    {
        Eigen::MatrixXd J = getCoMJacobian();
        owm_pimpl->vel_com = J.leftCols(6)*owm_pimpl->Troot+J.rightCols(owm_pimpl->nbInternalDofs)*owm_pimpl->dq;
    }
    else
        owm_pimpl->vel_com = getCoMJacobian()*owm_pimpl->dq;
    return owm_pimpl->vel_com;
}

const Eigen::Vector3d& OcraWbiModel::getCoMAngularVelocity() const
{
/*
    printf("Get COM Angular Velocity\n");
*/
    if (owm_pimpl->freeRoot)
    {
        Eigen::MatrixXd J = getCoMAngularJacobian();
        owm_pimpl->vel_com_angular = J.leftCols(6)*owm_pimpl->Troot+J.rightCols(owm_pimpl->nbInternalDofs)*owm_pimpl->dq;
    }
    else
        owm_pimpl->vel_com_angular = getCoMAngularJacobian()*owm_pimpl->dq;
    return owm_pimpl->vel_com_angular;
}

const Eigen::Vector3d& OcraWbiModel::getCoMJdotQdot() const
{
/*
    printf("Get COM JdotQdot\n");
*/
    Eigen::VectorXd dJdq(6);
    robot->computeDJdq(owm_pimpl->q.data(),owm_pimpl->Hroot_wbi,owm_pimpl->dq.data(),owm_pimpl->Troot_wbi.data(),wbi::iWholeBodyModel::COM_LINK_ID,dJdq.data());
    owm_pimpl->DJDq = dJdq.head(3);
    return owm_pimpl->DJDq;
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& OcraWbiModel::getCoMJacobian() const
{
/*
    printf("Get COM Jacobian\n");
*/
    robot->computeJacobian(owm_pimpl->q.data(), owm_pimpl->Hroot_wbi, wbi::iWholeBodyModel::COM_LINK_ID, owm_pimpl->J_com_rm.data());
    OcraWbiConversions::eigenRowMajorToColMajor(owm_pimpl->J_com_rm, owm_pimpl->J_com_full);

    getCoMPosition();

    if (owm_pimpl->freeRoot)
    {
        OcraWbiConversions::wbiToOcraCoMJacobian(owm_pimpl->J_com_full.topRows(COM_POS_DIM),owm_pimpl->J_com);
    }
    else
    {
        owm_pimpl->J_com = owm_pimpl->J_com_full.topRightCorner(COM_POS_DIM,owm_pimpl->nbInternalDofs);
    }

    return owm_pimpl->J_com;
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& OcraWbiModel::getCoMAngularJacobian() const
{
/*
    printf("Get COM Angular Jacobian\n");
*/
    robot->computeJacobian(owm_pimpl->q.data(), owm_pimpl->Hroot_wbi, wbi::iWholeBodyModel::COM_LINK_ID, owm_pimpl->J_com_rm_angular.data());
    OcraWbiConversions::eigenRowMajorToColMajor(owm_pimpl->J_com_rm_angular, owm_pimpl->J_com_full_angular);

    getCoMPosition();

    if (owm_pimpl->freeRoot)
    {
        OcraWbiConversions::wbiToOcraCoMJacobian(owm_pimpl->J_com_full_angular.bottomRows(COM_POS_DIM),owm_pimpl->J_com_angular);
    }
    else
    {
        owm_pimpl->J_com_angular = owm_pimpl->J_com_full_angular.bottomRightCorner(COM_POS_DIM,owm_pimpl->nbInternalDofs);
    }

    return owm_pimpl->J_com_angular;
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& OcraWbiModel::getCoMJacobianDot() const
{
/*
    printf("Get COM Jacobian Dot\n");
*/
    return owm_pimpl->DJ_com;
}

const Eigen::Displacementd& OcraWbiModel::getSegmentPosition(int index) const
{
/*
    printf("Get Segment Position : %d\n", index);
*/
    Frame H;
    // std::cout << "H_root in get Seg Position\n" << owm_pimpl->Hroot_wbi.toString() << std::endl;
    robot->computeH(owm_pimpl->q.data(),owm_pimpl->Hroot_wbi,index,H);
    OcraWbiConversions::wbiFrameToEigenDispd(H,owm_pimpl->segPosition[index]);
    return owm_pimpl->segPosition[index];
}

const Eigen::Twistd& OcraWbiModel::getSegmentVelocity(int index) const
{
/*
    printf("Get Segment Velocity : %d\n", index);
*/

    if (owm_pimpl->freeRoot)
    {
        Eigen::MatrixXd J = getSegmentJacobian(index);
        owm_pimpl->segVelocity[index] = J.leftCols(6)*owm_pimpl->Troot+J.rightCols(owm_pimpl->nbInternalDofs)*owm_pimpl->dq;
    }
    else
        owm_pimpl->segVelocity[index] = getSegmentJacobian(index)*owm_pimpl->dq;

    return owm_pimpl->segVelocity[index];
}

double OcraWbiModel::getSegmentMass(int index) const
{
/*
    printf("Get Segment Mass : %d\n", index);
*/
    return owm_pimpl->segMass[index];
}

const Eigen::Vector3d& OcraWbiModel::getSegmentCoM(int index) const
{
/*
    printf("Get Segment CoM : %d\n", index);
*/
    return owm_pimpl->segCoM[index];
}

const Eigen::Matrix<double,6,6>& OcraWbiModel::getSegmentMassMatrix(int index) const
{
/*
    printf("Get Segment Mass Matrix : %d\n", index);
*/
    return owm_pimpl->segMassMatrix[index];
}

const Eigen::Vector3d& OcraWbiModel::getSegmentMomentsOfInertia(int index) const
{
/*
    printf("Get Segment Moments of Inertia : %d\n", index);
*/
    return owm_pimpl->segMomentsOfInertia[index];
}

const Eigen::Rotation3d& OcraWbiModel::getSegmentInertiaAxes(int index) const
{
/*
    printf("Get Segment Inertia Axes : %d\n", index);
*/
    return owm_pimpl->segInertiaAxes[index];
}

//compute jacobian in segment frame
const Eigen::Matrix<double,6,Eigen::Dynamic>& OcraWbiModel::getSegmentJacobian(int index) const
{
    robot->computeJacobian(owm_pimpl->q.data(), owm_pimpl->Hroot_wbi, index, owm_pimpl->segJacobian_rm[index].data());

    OcraWbiConversions::eigenRowMajorToColMajor(owm_pimpl->segJacobian_rm[index], owm_pimpl->segJacobian_full[index]);
    OcraWbiConversions::wbiToOcraSegJacobian(owm_pimpl->segJacobian_full[index], owm_pimpl->segJacobian_full_ocra[index]);

    if (owm_pimpl->freeRoot)
    {
        owm_pimpl->segJacobian[index] = owm_pimpl->segJacobian_full_ocra[index];
//        const Eigen::Displacementd::Rotation3D& R_root = getFreeFlyerPosition().getRotation();
//        owm_pimpl->segJacobian[index].topLeftCorner(6,3)=owm_pimpl->segJacobian[index].topLeftCorner(6,3)*R_root.adjoint();
//        owm_pimpl->segJacobian[index].block<6,3>(0,3)=owm_pimpl->segJacobian[index].block<6,3>(0,3)*R_root.adjoint();
//        owm_pimpl->segJacobian[index].topLeftCorner(6,6)=owm_pimpl->segJacobian[index].topLeftCorner(6,6)*getFreeFlyerPosition().adjoint();


//        const Eigen::Displacementd& H_seg_root = getSegmentPosition(index).inverse()*getFreeFlyerPosition();
//        owm_pimpl->segJacobian[index].topLeftCorner(6,6) = H_seg_root.adjoint();
//        owm_pimpl->segJacobian[index].topLeftCorner(3,3)= R_seg_root.adjoint();
//        owm_pimpl->segJacobian[index].block<3,3>(0,3).setZero();
//        owm_pimpl->segJacobian[index].block<3,3>(3,0).setZero();
//        owm_pimpl->segJacobian[index].block<3,3>(3,3)= R_seg_root.adjoint();


    }
    else
        owm_pimpl->segJacobian[index] = owm_pimpl->segJacobian_full_ocra[index].rightCols(owm_pimpl->nbInternalDofs);

//    /**
//    * We must project the jacobian in the segment frame orientation in order to work with the controller.
//    */
//    owm_pimpl->segJacobian[index] = getSegmentPosition(index).inverse().adjoint()*owm_pimpl->segJacobian[index];
//    const Eigen::Displacementd::Rotation3D& R_seg_w = getSegmentPosition(index).getRotation().inverse();
//    owm_pimpl->segJacobian[index].topRows(3) = R_seg_w.adjoint()*owm_pimpl->segJacobian[index].topRows(3);
//    owm_pimpl->segJacobian[index].bottomRows(3) = R_seg_w.adjoint()*owm_pimpl->segJacobian[index].bottomRows(3);

//    std::cout<<"jac"<<index<<"="<<owm_pimpl->segJacobian[index].topLeftCorner(6,6)<<std::endl;
//    owm_pimpl->segJacobian[index].topRightCorner(3,owm_pimpl->nbInternalDofs) = R_seg_w.adjoint()*owm_pimpl->segJacobian[index].topRightCorner(3,owm_pimpl->nbInternalDofs);
//    owm_pimpl->segJacobian[index].bottomRightCorner(3,owm_pimpl->nbInternalDofs) = R_seg_w.adjoint()*owm_pimpl->segJacobian[index].bottomRightCorner(3,owm_pimpl->nbInternalDofs);


    return owm_pimpl->segJacobian[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OcraWbiModel::getSegmentJdot(int index) const
{
/*
    printf("Get Segment Jacobian Dot : %d\n", index);
*/
    return owm_pimpl->segJdot[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OcraWbiModel::getJointJacobian(int index) const
{
/*
    printf("Get Joint Jacobian Dot : %d\n", index);
*/
    owm_pimpl->segJointJacobian[index] = getSegmentJacobian(index);

    return owm_pimpl->segJointJacobian[index];
}

const Eigen::Twistd& OcraWbiModel::getSegmentJdotQdot(int index) const
{
/*
    printf("Get Segment JdotQdot : %d\n", index);
*/
    Eigen::Twistd Tseg;
    robot->computeDJdq(owm_pimpl->q.data(),owm_pimpl->Hroot_wbi,owm_pimpl->dq.data(),owm_pimpl->Troot_wbi.data(),index,Tseg.data());

    OcraWbiConversions::wbiToOcraTwistVector(Tseg, owm_pimpl->segJdotQdot[index]);

//    const Eigen::Displacementd::Rotation3D& R_seg_w = getSegmentPosition(index).getRotation().inverse();
////    T = H.inverse().adjoint()*T;
//    owm_pimpl->segJdotQdot[index].bottomRows(3) = R_seg_w.adjoint()*owm_pimpl->segJdotQdot[index].bottomRows(3);
//    owm_pimpl->segJdotQdot[index].topRows(3) = R_seg_w.adjoint()*owm_pimpl->segJdotQdot[index].topRows(3);
    return owm_pimpl->segJdotQdot[index];
}

void OcraWbiModel::doSetJointPositions(const Eigen::VectorXd& q)
{
/*
    printf("set joint pos to :\n");
    std::cout << q.transpose() << std::endl;
*/
    owm_pimpl->q = q;
}

void OcraWbiModel::doSetJointVelocities(const Eigen::VectorXd& dq)
{
/*
    printf("set joint vel to :\n");
    std::cout << dq.transpose() << std::endl;
*/
    owm_pimpl->dq = dq;
}

void OcraWbiModel::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
    owm_pimpl->Hroot = Hroot;
    OcraWbiConversions::eigenDispdToWbiFrame(owm_pimpl->Hroot, owm_pimpl->Hroot_wbi);
}

void OcraWbiModel::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
    owm_pimpl->Troot = Troot;
    OcraWbiConversions::ocraToWbiTwistVector(owm_pimpl->Troot, owm_pimpl->Troot_wbi);
}

int OcraWbiModel::doGetSegmentIndex(const std::string& name) const
{
    int id;
    bool ok = robot->getFrameList().idToIndex(name.c_str(), id);
    return id;
}

int OcraWbiModel::doGetDofIndex(const std::string &name) const
{
    int id;
    bool ok = robot->getJointList().idToIndex(name.c_str(), id);
    return id;
}

const std::string& OcraWbiModel::doGetDofName(int index) const
{
    wbi::ID dofID; //wbi::IDList jList =
    bool res = robot->getJointList().indexToID(index, dofID);
    return dofID.toString();
    // throw std::runtime_error("[OcraWbiModel::doGetDofName] This function was not overriden for a specific model");
}


const std::string& OcraWbiModel::doGetSegmentName(int index) const
{
    wbi::ID segID;
    bool res = robot->getFrameList().indexToID(index, segID);
    return segID.toString();
}

const std::string OcraWbiModel::doSegmentName(const std::string& name) const
{
    // Return segmentName directly
    return name;
}

const std::string OcraWbiModel::doDofName(const std::string& name) const
{
    // Return dofName directly
    return name;
}


void OcraWbiModel::doSetState(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot)
{
    // Do nothing
}

void OcraWbiModel::doSetState(const Eigen::Displacementd& H_root, const Eigen::VectorXd& q, const Eigen::Twistd& T_root, const Eigen::VectorXd& q_dot)
{
    // Do nothing
}

void OcraWbiModel::printAllData()
{
    std::cout<<"nbSegments:\n";
    std::cout<<nbSegments()<<"\n";

    std::cout<<"nbDofs:\n";
    std::cout<<nbDofs()<<std::endl;

    std::cout<<"nbInternalDofs:\n";
    std::cout<<nbInternalDofs()<<std::endl;

    std::cout<<"actuatedDofs:\n";
    std::cout<<getActuatedDofs()<<"\n";

    std::cout<<"lowerLimits:\n";
    std::cout<<getJointLowerLimits()<<"\n";

    std::cout<<"upperLimits:\n";
    std::cout<<getJointUpperLimits()<<"\n";

    std::cout<<"q:\n";
    std::cout<<getJointPositions().transpose()<<"\n";

    std::cout<<"dq:\n";
    std::cout<<getJointVelocities().transpose()<<"\n";

    std::cout<<"Hroot:\n";
    std::cout<<getFreeFlyerPosition()<<"\n";

    std::cout<<"Troot:\n";
//    std::cout<<getFreeFlyerVelocity().transpose()<<"\n";

    std::cout<<"total_mass:\n";
    std::cout<<getMass()<<"\n";

    std::cout<<"M:\n";
    std::cout<<getInertiaMatrix()<<"\n";

    std::cout<<"Minv:\n";
    std::cout<<getInertiaMatrixInverse()<<"\n";

    std::cout<<"B:\n";
    std::cout<<getDampingMatrix()<<"\n";

    std::cout<<"n:\n";
    std::cout<<getNonLinearTerms()<<"\n";

    std::cout<<"g:\n";
    std::cout<<getGravityTerms()<<"\n";

    std::cout<<"l:\n";
    std::cout<<getLinearTerms()<<"\n";

    std::cout<<"comPosition:\n";
    std::cout<<getCoMPosition().transpose()<<"\n";

    std::cout<<"comVelocity:\n";
    std::cout<<getCoMVelocity().transpose()<<"\n";

    std::cout<<"comJdotQdot:\n";
    std::cout<<getCoMJdotQdot().transpose()<<"\n";

    std::cout<<"comJacobian:\n";
    std::cout<<getCoMJacobian()<<"\n";

    std::cout<<"comJacobianDot:\n";
    std::cout<<getCoMJacobianDot()<<"\n";



    for (int idx=0; idx<nbSegments(); idx++)
    {
        std::cout<<"segPosition "<<idx<<":\n";
        std::cout<<getSegmentPosition(idx)<<"\n";

        std::cout<<"segVelocity "<<idx<<":\n";
        std::cout<<getSegmentVelocity(idx)<<"\n";

        std::cout<<"segMass "<<idx<<":\n";
        std::cout<<getSegmentMass(idx)<<"\n";

        std::cout<<"segCoM "<<idx<<":\n";
        std::cout<<getSegmentCoM(idx)<<"\n";

        std::cout<<"segMassMatrix "<<idx<<":\n";
        std::cout<<getSegmentMassMatrix(idx)<<"\n";

        std::cout<<"segMomentsOfInertia "<<idx<<":\n";
        std::cout<<getSegmentMomentsOfInertia(idx)<<"\n";

        std::cout<<"segInertiaAxes "<<idx<<":\n";
        std::cout<<getSegmentInertiaAxes(idx)<<"\n";

        std::cout<<"segJacobian "<<idx<<":\n";
        std::cout<<getSegmentJacobian(idx)<<"\n";

        std::cout<<"segJdot "<<idx<<":\n";
        std::cout<<getSegmentJdot(idx)<<"\n";

        std::cout<<"segJointJacobian "<<idx<<":\n";
        std::cout<<getJointJacobian(idx)<<"\n";

        std::cout<<"segJdotQdot "<<idx<<":\n";
        std::cout<<getSegmentJdotQdot(idx).transpose()<<"\n";

    }

}
