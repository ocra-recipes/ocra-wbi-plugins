#ifndef ICUB_CONTROLLER_SERVER_H
#define ICUB_CONTROLLER_SERVER_H

#include <wbi/wbi.h>
#include <ocra-recipes/ControllerServer.h>
#include <Eigen/Dense>
#include <ocra-icub-server/OcraWbiModel.h>

class IcubControllerServer : public ocra_recipes::ControllerServer
{
public:
    IcubControllerServer();
    IcubControllerServer(   std::shared_ptr<wbi::wholeBodyInterface> robot,
                            std::string icubName,
                            const bool usingFloatingBase,
                            const ocra_recipes::CONTROLLER_TYPE ctrlType=ocra_recipes::WOCRA_CONTROLLER,
                            const bool usingInterprocessCommunication=true
                        );
    virtual ~IcubControllerServer();

    virtual std::shared_ptr<Model> loadRobotModel();

    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root);

private:
    std::shared_ptr<wbi::wholeBodyInterface> wbi; /*!< The WBI used to talk to the robot. */
    std::string robotName;
    bool isFloatingBase;
    static const int ALL_JOINTS = -1;
    int nDoF;

    Eigen::VectorXd wbi_H_root_Vector;
    Eigen::VectorXd wbi_T_root_Vector;
    wbi::Frame wbi_H_root;


};

#endif
