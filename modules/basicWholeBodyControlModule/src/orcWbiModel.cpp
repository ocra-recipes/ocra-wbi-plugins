
#include "orcWbiModel.h"
#include "orcWbiUtil.h"

#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/sig/Matrix.h>
#include <map>
#include <vector>
#include <iostream>

#define ALL_JOINTS -1
#define FREE_ROOT_DOF 6
#define COM_POS_DIM 3
#define TRANS_ROT_DIM 6


typedef  Eigen::Displacementd::AdjointMatrix  AdjointMatrix;

struct orcWbiModel::orcWbiModel_pimpl
{
public:
    int                                                     nbDofs;
    int                                                     nbInternalDofs; // nbDofs + FREE_ROOT_DOF if free root, otherwise the same as nbDofs

    int                                                     nbSegments; // nbInternalDofs+1?
    Eigen::VectorXd                                         actuatedDofs; // which joints are actuated
    Eigen::VectorXd                                         lowerLimits; // lower q of joints
    Eigen::VectorXd                                         upperLimits; // upper q of joints
    Eigen::VectorXd                                         q; // state variable
    Eigen::VectorXd                                         dq; // derivative of q
    Eigen::Displacementd                                    Hroot; // translation of root
    Eigen::Twistd                                           Troot; // twist of root (velocity)
    Eigen::MatrixXd                                         M; // Mass inertia matrix (col major for ORC control)
    MatrixXdRm                                              M_rm; // Mass inertia matrix (row major for WBI)
    Eigen::MatrixXd                                         Minv; // Inverse of mass inertia matrix (col major for ORC control)
    Eigen::MatrixXd                                         B; // Not set, set to ZERO for now (col major for ORC control)
    Eigen::VectorXd                                         n; // non-linear terms in EOM (set as coriolis/centrifugal effects)
    Eigen::VectorXd                                         l; // linear terms in EOM (set this to be zero)
    Eigen::VectorXd                                         g; // gravity term in EOM
    double                                                  total_mass;
    Eigen::Vector3d                                         pos_com; // COM position
    Eigen::Vector3d                                         vel_com; // COM velocity
    Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>        J_com; // Jacobian matrix (col major for ORC control)
    Eigen::MatrixXd                                         J_com_cm; // Jacobian matrix (col major MatrixXd for ORC control)
    MatrixXdRm                                              J_com_rm; // Jacobian matrix (row major for WBI)
    Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>        DJ_com; // derivative of J
    Eigen::MatrixXd                                         DJ_com_cm; // derivative of J
    MatrixXdRm                                              DJ_com_rm; // derivative of J
    Eigen::Vector3d                                         DJDq;

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
    std::vector< Eigen::MatrixXd >                                      segJacobian_cm; 
    std::vector< MatrixXdRm >                                           segJacobian_rm; 
    std::vector< Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic> >   segJdot; // not set
    std::vector< Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic> >   segJointJacobian; // not set
    std::vector< Eigen::Twistd >                            segJdotQdot;
    std::map< std::string, int >                            segIndexFromName;
    std::vector< std::string >                              segNameFromIndex;
    Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>      Jroot; 
    Eigen::Matrix<double,TRANS_ROT_DIM,Eigen::Dynamic>      dJroot;
    
};

//=================================  Class methods  =================================//
orcWbiModel::orcWbiModel(const std::string& robotName, const int robotNumDOF, wholeBodyInterface* _wbi, const bool freeRoot)
:orc::Model(robotName, freeRoot?robotNumDOF+FREE_ROOT_DOF:robotNumDOF, freeRoot),robot(_wbi),owm_pimpl(new orcWbiModel_pimpl)
{
    // Initialise some constant variables

    // THIS GETS FROM WBI ROBOT
    //owm_pimpl->nbDofs = freeRoot?robot->getDoFs()+FREE_ROOT_DOF:robot->getDoFs();
    //owm_pimpl->nbInternalDofs = robot->getDoFs();
    // THIS GETS FROM ORC MODEL
    owm_pimpl->nbDofs = nbDofs();
    owm_pimpl->nbInternalDofs = nbInternalDofs();

    // Need to FIX THIS TO GET THE VALUE PROPERLY!
    owm_pimpl->nbSegments = owm_pimpl->nbInternalDofs + 1;
    // Ones to indicate that all joints are actuated 
    owm_pimpl->actuatedDofs = Eigen::VectorXd::Ones(owm_pimpl->nbDofs);

    // Setup and get lower/upper joint limits
    owm_pimpl->lowerLimits = Eigen::VectorXd::Ones(owm_pimpl->nbInternalDofs);
    owm_pimpl->upperLimits = Eigen::VectorXd::Ones(owm_pimpl->nbInternalDofs);

    robot->getJointLimits(owm_pimpl->lowerLimits.data(), owm_pimpl->upperLimits.data(), ALL_JOINTS);
    
    // Setup joint states
    owm_pimpl->q = Eigen::VectorXd::Zero(owm_pimpl->nbInternalDofs);
    owm_pimpl->dq = Eigen::VectorXd::Zero(owm_pimpl->nbInternalDofs);

    // Setup mass matrix, damping matrix, c and g terms 
    owm_pimpl->M.resize(owm_pimpl->nbDofs, owm_pimpl->nbDofs);
    owm_pimpl->M_rm.resize(owm_pimpl->nbDofs, owm_pimpl->nbDofs);
    owm_pimpl->B = Eigen::MatrixXd::Zero(owm_pimpl->nbDofs, owm_pimpl->nbDofs);
    owm_pimpl->n.resize(owm_pimpl->nbDofs);
    owm_pimpl->l = Eigen::VectorXd::Zero(owm_pimpl->nbDofs);
    owm_pimpl->g.resize(owm_pimpl->nbDofs);

    MatrixXdRm M_rm_total_mass(38,38);
    robot->computeMassMatrix(owm_pimpl->q.data(), wbi::Frame(), M_rm_total_mass.data());
    owm_pimpl->total_mass = M_rm_total_mass(0,0);
    
    owm_pimpl->J_com.resize(COM_POS_DIM, owm_pimpl->nbDofs);
    owm_pimpl->J_com_cm.resize(COM_POS_DIM, owm_pimpl->nbDofs);
    owm_pimpl->J_com_rm.resize(COM_POS_DIM, owm_pimpl->nbDofs);
}

orcWbiModel::~orcWbiModel()
{
    
}

int orcWbiModel::nbSegments() const
{
    return owm_pimpl->nbSegments;
}

const Eigen::VectorXd& orcWbiModel::getActuatedDofs() const
{
    return owm_pimpl->actuatedDofs;
}

const Eigen::VectorXd& orcWbiModel::getJointLowerLimits() const
{
    return owm_pimpl->lowerLimits;
}

const Eigen::VectorXd& orcWbiModel::getJointUpperLimits() const
{
    return owm_pimpl->upperLimits;
}

const Eigen::VectorXd& orcWbiModel::getJointPositions() const
{
    bool res = robot->getEstimates(ESTIMATE_JOINT_POS, owm_pimpl->q.data());
    return owm_pimpl->q;
}

const Eigen::VectorXd& orcWbiModel::getJointVelocities() const
{
    bool res = robot->getEstimates(ESTIMATE_JOINT_VEL, owm_pimpl->dq.data());
    return owm_pimpl->dq;
}

const Eigen::Displacementd& orcWbiModel::getFreeFlyerPosition() const
{
    return owm_pimpl->Hroot;
}

const Eigen::Twistd& orcWbiModel::getFreeFlyerVelocity() const
{
    return owm_pimpl->Troot;
}

const Eigen::MatrixXd& orcWbiModel::getInertiaMatrix() const
{
    Eigen::VectorXd q = getJointPositions();
    bool res = robot->computeMassMatrix(q.data(), wbi::Frame(), owm_pimpl->M_rm.data());
    orcWbiConversions::eigenRowMajorToColMajor(owm_pimpl->M_rm, owm_pimpl->M);
    
    std::cout << "SIZE ROWS : " << owm_pimpl->M_rm.rows() << " COLS : " << owm_pimpl->M_rm.cols() << std::endl;
    return owm_pimpl->M;
}

const Eigen::MatrixXd& orcWbiModel::getInertiaMatrixInverse() const
{
    owm_pimpl->Minv = owm_pimpl->M.inverse();
    return owm_pimpl->Minv;
}

const Eigen::MatrixXd& orcWbiModel::getDampingMatrix() const
{
}

const Eigen::VectorXd& orcWbiModel::getNonLinearTerms() const
{
}

const Eigen::VectorXd& orcWbiModel::getLinearTerms() const
{
}

const Eigen::VectorXd& orcWbiModel::getGravityTerms() const
{
}

double orcWbiModel::getMass() const
{
    return owm_pimpl->total_mass;
}

const Eigen::Vector3d& orcWbiModel::getCoMPosition() const
{
    Eigen::VectorXd q = getJointPositions();
    Eigen::Displacementd Hroot = getFreeFlyerPosition();
    wbi::Frame Hbase,H;
    orcWbiConversions::eigenDispdToWbiFrame(Hroot,Hbase);
    robot->computeH(q.data(),Hbase,wbi::iWholeBodyModel::COM_LINK_ID,H);
    Eigen::Displacementd Hcom;
    orcWbiConversions::wbiFrameToEigenDispd(H,Hcom);
    owm_pimpl->pos_com = Hcom.getTranslation();
    return owm_pimpl->pos_com;
}

const Eigen::Vector3d& orcWbiModel::getCoMVelocity() const
{
    owm_pimpl->vel_com = getCoMJacobian()*getJointVelocities();
    return owm_pimpl->vel_com;
}

const Eigen::Vector3d& orcWbiModel::getCoMJdotQdot() const
{
    Eigen::VectorXd q = getJointPositions();
    Eigen::VectorXd dq = getJointVelocities();
    Eigen::Displacementd Hroot = getFreeFlyerPosition();
    wbi::Frame Hbase,H;
    orcWbiConversions::eigenDispdToWbiFrame(Hroot,Hbase);
    Eigen::Twistd Troot = getFreeFlyerVelocity();
    robot->computeDJdq(q.data(),Hbase,dq.data(),Troot.data(),wbi::iWholeBodyModel::COM_LINK_ID,owm_pimpl->DJDq.data());
    return owm_pimpl->DJDq;
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& orcWbiModel::getCoMJacobian() const
{

    Eigen::VectorXd q = getJointPositions();
    Eigen::Displacementd Hroot = getFreeFlyerPosition();
    wbi::Frame Hbase, H;
    orcWbiConversions::eigenDispdToWbiFrame(Hroot, Hbase);
    robot->computeJacobian(q.data(), Hbase, wbi::iWholeBodyModel::COM_LINK_ID, owm_pimpl->J_com_rm.data());
    orcWbiConversions::eigenRowMajorToColMajor(owm_pimpl->J_com_rm, owm_pimpl->J_com_cm);
    owm_pimpl->J_com = owm_pimpl->J_com_cm;
    return owm_pimpl->J_com;
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& orcWbiModel::getCoMJacobianDot() const
{
    owm_pimpl->DJ_com = Eigen::MatrixXd::Zero(COM_POS_DIM, owm_pimpl->nbDofs);
    return owm_pimpl->DJ_com;
}

const Eigen::Displacementd& orcWbiModel::getSegmentPosition(int index) const
{
    Eigen::VectorXd q = getJointPositions();
    Eigen::Displacementd Hroot = getFreeFlyerPosition();
    wbi::Frame Hbase,H;
    orcWbiConversions::eigenDispdToWbiFrame(Hroot,Hbase);
    robot->computeH(q.data(),Hbase,index,H);

    orcWbiConversions::wbiFrameToEigenDispd(H,owm_pimpl->segPosition[index]);

    return owm_pimpl->segPosition[index];
}

const Eigen::Twistd& orcWbiModel::getSegmentVelocity(int index) const
{
    owm_pimpl->segVelocity[index] = getSegmentJacobian(index)*getJointVelocities();
    return owm_pimpl->segVelocity[index];
}

double orcWbiModel::getSegmentMass(int index) const
{
    owm_pimpl->segMass[index] = 0.0;
    return owm_pimpl->segMass[index];
}

const Eigen::Vector3d& orcWbiModel::getSegmentCoM(int index) const
{
    owm_pimpl->segCoM[index] = Eigen::Vector3d::Zero();
    return owm_pimpl->segCoM[index];
}

const Eigen::Matrix<double,6,6>& orcWbiModel::getSegmentMassMatrix(int index) const
{
    owm_pimpl->segMassMatrix[index] = Eigen::MatrixXd::Zero(6,6);
    return owm_pimpl->segMassMatrix[index];
}

const Eigen::Vector3d& orcWbiModel::getSegmentMomentsOfInertia(int index) const
{
    owm_pimpl->segMomentsOfInertia[index] = Eigen::Vector3d::Zero();
    return owm_pimpl->segMomentsOfInertia[index];
}

const Eigen::Rotation3d& orcWbiModel::getSegmentInertiaAxes(int index) const
{
    owm_pimpl->segInertiaAxes[index] = Eigen::Rotation3d::Identity();
    return owm_pimpl->segInertiaAxes[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcWbiModel::getSegmentJacobian(int index) const
{
    Eigen::VectorXd q = getJointPositions();
    Eigen::Displacementd Hroot = getFreeFlyerPosition();
    wbi::Frame Hbase, H;
    orcWbiConversions::eigenDispdToWbiFrame(Hroot, Hbase);
    robot->computeJacobian(q.data(), Hbase, index, owm_pimpl->segJacobian_rm[index].data());
    orcWbiConversions::eigenRowMajorToColMajor(owm_pimpl->segJacobian_rm[index], owm_pimpl->segJacobian_cm[index]);
    owm_pimpl->segJacobian[index] = owm_pimpl->segJacobian_cm[index];

    return owm_pimpl->segJacobian[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcWbiModel::getSegmentJdot(int index) const
{
    owm_pimpl->segJdot[index] = Eigen::MatrixXd::Zero(6, owm_pimpl->nbDofs);
    return owm_pimpl->segJdot[index];
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcWbiModel::getJointJacobian(int index) const
{
}

const Eigen::Twistd& orcWbiModel::getSegmentJdotQdot(int index) const
{
    Eigen::VectorXd q = getJointPositions();
    Eigen::VectorXd dq = getJointVelocities();
    Eigen::Displacementd Hroot = getFreeFlyerPosition();
    wbi::Frame Hbase,H;
    orcWbiConversions::eigenDispdToWbiFrame(Hroot,Hbase);
    Eigen::Twistd Troot = getFreeFlyerVelocity();
    robot->computeDJdq(q.data(),Hbase,dq.data(),Troot.data(),index,owm_pimpl->segJdotQdot[index].data());
    return owm_pimpl->segJdotQdot[index];
}

void orcWbiModel::doSetJointPositions(const Eigen::VectorXd& q)
{
}

void orcWbiModel::doSetJointVelocities(const Eigen::VectorXd& dq)
{
}

void orcWbiModel::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
}

void orcWbiModel::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
}

int orcWbiModel::doGetSegmentIndex(const std::string& name) const
{
}

const std::string& orcWbiModel::doGetSegmentName(int index) const
{
}

void orcWbiModel::printAllData()
{
    std::cout<<"nbSegments:\n";
    std::cout<<nbSegments()<<"\n";

    std::cout<<"nbDofs:\n";
    std::cout<<nbDofs()<<std::endl;
    
    std::cout<<"nbInternalDofs:\n";
    std::cout<<nbInternalDofs()<<std::endl;

    std::cout<<"actuatedDofs:\n";
    std::cout<<getActuatedDofs()<<"\n";
    
/*
    std::cout<<"lowerLimits:\n";
    std::cout<<getJointLowerLimits()<<"\n";
    
    std::cout<<"upperLimits:\n";
    std::cout<<getJointUpperLimits()<<"\n";
*/
    
    std::cout<<"q:\n";
    std::cout<<getJointPositions()<<"\n";
    
    std::cout<<"dq:\n";
    std::cout<<getJointVelocities()<<"\n";
    
    std::cout<<"Hroot:\n";
    std::cout<<getFreeFlyerPosition()<<"\n";
    
    std::cout<<"Troot:\n";
    std::cout<<getFreeFlyerVelocity()<<"\n";


    std::cout<<"M:\n";
    std::cout<<getInertiaMatrix()<<"\n";
    
/*
    std::cout<<"Minv:\n";
    std::cout<<getInertiaMatrixInverse()<<"\n";
*/
/*
    std::cout<<"total_mass:\n";
    std::cout<<getMass()<<"\n";
    
    std::cout<<"comPosition:\n";
    std::cout<<getCoMPosition()<<"\n";
    
    std::cout<<"comVelocity:\n";
    std::cout<<getCoMVelocity()<<"\n";
    
    std::cout<<"comJdotQdot:\n";
    std::cout<<getCoMJdotQdot()<<"\n";
    
    std::cout<<"comJacobian:\n";
    std::cout<<getCoMJacobian()<<"\n";
    
    std::cout<<"comJacobianDot:\n";
    std::cout<<getCoMJacobianDot()<<"\n";
    
    
    
    std::cout<<"B:\n";
    std::cout<<getDampingMatrix()<<"\n";
    
    std::cout<<"n:\n";
    std::cout<<getNonLinearTerms()<<"\n";
    
    std::cout<<"l:\n";
    std::cout<<getLinearTerms()<<"\n";
    
    std::cout<<"g:\n";
    std::cout<<getGravityTerms()<<"\n";
    
    
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
        std::cout<<getSegmentJdotQdot(idx)<<"\n";
    
    }
*/
    
}


