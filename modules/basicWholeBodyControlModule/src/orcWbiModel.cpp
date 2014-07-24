
#include "orcWbiModel.h"

#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/sig/Matrix.h>
#include <map>
#include <vector>
#include <iostream>

#define ALL_JOINTS -1
#define FREE_ROOT_DOF 6

typedef  Eigen::Displacementd::AdjointMatrix  AdjointMatrix;

struct orcWbiModel::orcWbiModel_pimpl
{
public:
    int                                                     nbDofs;
    int                                                     nbInternalDofs; // nbDofs + FREE_ROOT_DOF if free root, otherwise the same as nbDofs

    int                                                     nbSegments;
    Eigen::VectorXd                                         actuatedDofs;
    Eigen::VectorXd                                         lowerLimits;
    Eigen::VectorXd                                         upperLimits;
    Eigen::VectorXd                                         q;
    Eigen::VectorXd                                         dq;
    Eigen::Displacementd                                    Hroot;
    Eigen::Twistd                                           Troot;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>       M;
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
    owm_pimpl->q.resize(owm_pimpl->nbInternalDofs);
    owm_pimpl->dq.resize(owm_pimpl->nbInternalDofs);

    // Setup mass matrix 
    owm_pimpl->M.resize(owm_pimpl->nbDofs, owm_pimpl->nbDofs);
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
    bool res = robot->computeMassMatrix(q.data(), wbi::Frame(), owm_pimpl->M.data());

    return owm_pimpl->M;
}

const Eigen::MatrixXd& orcWbiModel::getInertiaMatrixInverse() const
{
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
}

const Eigen::Vector3d& orcWbiModel::getCoMPosition() const
{
}

const Eigen::Vector3d& orcWbiModel::getCoMVelocity() const
{
}

const Eigen::Vector3d& orcWbiModel::getCoMJdotQdot() const
{
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& orcWbiModel::getCoMJacobian() const
{
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& orcWbiModel::getCoMJacobianDot() const
{
}

const Eigen::Displacementd& orcWbiModel::getSegmentPosition(int index) const
{
}

const Eigen::Twistd& orcWbiModel::getSegmentVelocity(int index) const
{
}

double orcWbiModel::getSegmentMass(int index) const
{
}

const Eigen::Vector3d& orcWbiModel::getSegmentCoM(int index) const
{
}

const Eigen::Matrix<double,6,6>& orcWbiModel::getSegmentMassMatrix(int index) const
{
}

const Eigen::Vector3d& orcWbiModel::getSegmentMomentsOfInertia(int index) const
{
}

const Eigen::Rotation3d& orcWbiModel::getSegmentInertiaAxes(int index) const
{
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcWbiModel::getSegmentJacobian(int index) const
{
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcWbiModel::getSegmentJdot(int index) const
{
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcWbiModel::getJointJacobian(int index) const
{
}

const Eigen::Twistd& orcWbiModel::getSegmentJdotQdot(int index) const
{
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
    
    
    std::cout<<"Minv:\n";
    std::cout<<getInertiaMatrixInverse()<<"\n";
    
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


