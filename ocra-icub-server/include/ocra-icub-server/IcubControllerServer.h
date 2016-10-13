#ifndef ICUB_CONTROLLER_SERVER_H
#define ICUB_CONTROLLER_SERVER_H

#include <wbi/wbi.h>
#include <ocra-recipes/ControllerServer.h>
#include <Eigen/Dense>
#include <ocra-icub/OcraWbiModel.h>
#include <iDynTree/Estimation/SimpleLeggedOdometry.h>

class IcubControllerServer : public ocra_recipes::ControllerServer
{
public:
    // IcubControllerServer();
    IcubControllerServer(   std::shared_ptr<wbi::wholeBodyInterface> robot,
                            std::string icubName,
                            const bool usingFloatingBase,
                            const ocra_recipes::CONTROLLER_TYPE ctrlType=ocra_recipes::WOCRA_CONTROLLER,
                            const ocra_recipes::SOLVER_TYPE solver=ocra_recipes::QUADPROG,
                            const bool usingInterprocessCommunication=true,
                            const bool useOdometry=false
                        );
    virtual ~IcubControllerServer();

    virtual ocra::Model::Ptr loadRobotModel();

    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root);
    
    // Odometry related methods
    bool initializeOdometry(std::string model_file, std::string initialFixedFrame);
    std::vector<std::string> getCanonical_iCubJoints();
    // Not in the virtual class
    void rootFrameVelocity(Eigen::VectorXd& q,
                           Eigen::VectorXd& qd,
                           double           regularization,
                           int              LEFT_FOOT_CONTACT,
                           int              RIGHT_FOOT_CONTACT,
                           Eigen::VectorXd& twist);
    void pinv(Eigen::MatrixXd mat, Eigen::MatrixXd& pinvmat, double pinvtoler=1.0e-6) const;
private:
    std::shared_ptr<wbi::wholeBodyInterface> wbi; /*!< The WBI used to talk to the robot. */
    std::string robotName;
    bool isFloatingBase;
    bool useOdometry;
    static const int ALL_JOINTS = -1;
    int nDoF;

    Eigen::VectorXd wbi_H_root_Vector;
    Eigen::VectorXd wbi_T_root_Vector;
    wbi::Frame wbi_H_root;
    
    iDynTree::SimpleLeggedOdometry odometry;
    
};

#endif
