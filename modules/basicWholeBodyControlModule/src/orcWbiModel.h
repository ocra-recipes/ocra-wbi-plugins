

#ifndef ORCWBIMODEL_H
#define ORCWBIMODEL_H

#include "orc/control/Model.h"
#include <wbi/wbi.h>

using namespace wbi;

class orcWbiModel: public orc::Model
{
public:

//===========================Constructor/Destructor===========================//
    orcWbiModel(const std::string& robotName, const int robotNumDOF, wholeBodyInterface* wbi, const bool freeRoot);
    virtual ~orcWbiModel();

//=============================General functions==============================//
    virtual int                          nbSegments               () const;
    virtual const Eigen::VectorXd&       getActuatedDofs          () const;
    virtual const Eigen::VectorXd&       getJointLowerLimits      () const;
    virtual const Eigen::VectorXd&       getJointUpperLimits      () const;
    virtual const Eigen::VectorXd&       getJointPositions        () const;
    virtual const Eigen::VectorXd&       getJointVelocities       () const;
    virtual const Eigen::Displacementd&  getFreeFlyerPosition     () const;
    virtual const Eigen::Twistd&         getFreeFlyerVelocity     () const;

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

protected:

//===========================Update state functions===========================//
    virtual void                doSetJointPositions     (const Eigen::VectorXd& q);
    virtual void                doSetJointVelocities    (const Eigen::VectorXd& dq);
    virtual void                doSetFreeFlyerPosition  (const Eigen::Displacementd& Hroot);
    virtual void                doSetFreeFlyerVelocity  (const Eigen::Twistd& Troot);

//============================Index name functions============================//
    virtual int                 doGetSegmentIndex       (const std::string& name) const;
    virtual const std::string&  doGetSegmentName        (int index) const;



private:
    wholeBodyInterface* robot; // Access to wholeBodyInterface
    struct orcWbiModel_pimpl;
    boost::shared_ptr<orcWbiModel_pimpl> owm_pimpl; // where all internal data are saved
};

/*
extern "C"
{
    orc::Model* CreateICub(const std::string& robotName)
    {
        return new orcWbiModel(robotName);
    }
}
*/

#endif
