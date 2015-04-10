#ifndef OCRAWBIMODEL_H
#define OCRAWBIMODEL_H

#include "wocra/Models/wOcraModel.h"
#include <wbi/wbi.h>

using namespace wbi;

class ocraWbiModel: public wocra::wOcraModel
{
public:

//===========================Constructor/Destructor===========================//
    ocraWbiModel(const std::string& robotName, const int robotNumDOF, wholeBodyInterface* wbi, const bool freeRoot);
    virtual ~ocraWbiModel();

//=============================General functions==============================//
    virtual int                          nbSegments               () const;
    virtual const Eigen::VectorXd&       getActuatedDofs          () const;
    virtual const Eigen::VectorXd&       getJointLowerLimits      () const;
    virtual const Eigen::VectorXd&       getJointUpperLimits      () const;
    virtual const Eigen::VectorXd&       getJointPositions        () const;
    virtual const Eigen::VectorXd&       getJointVelocities       () const;
    virtual const std::string&           getJointName             (int index) const;
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

    // Set state to be used from WBI
    void wbiSetState(const wbi::Frame& H_root, const Eigen::VectorXd& q, const Eigen::Twistd& T_root, const Eigen::VectorXd& q_dot);

protected:

//===========================Update state functions===========================//
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
    wholeBodyInterface* robot; // Access to wholeBodyInterface
    struct ocraWbiModel_pimpl;
    boost::shared_ptr<ocraWbiModel_pimpl> owm_pimpl; // where all internal data are saved
};

/*
extern "C"
{
    ocra::Model* CreateICub(const std::string& robotName)
    {
        return new ocraWbiModel(robotName);
    }
}
*/

#endif
