#include <ocra-icub-server/IcubControllerServer.h>

IcubControllerServer::IcubControllerServer()
{

}

IcubControllerServer::IcubControllerServer( std::shared_ptr<wbi::wholeBodyInterface> robot,
                                            std::string icubName,
                                            const bool usingFloatingBase,
                                            const ocra_recipes::CONTROLLER_TYPE ctrlType,
                                            const bool usingInterprocessCommunication
                                        )
: ocra_recipes::ControllerServer(ctrlType, usingInterprocessCommunication)
, wbi(robot)
, robotName(icubName)
, isFloatingBase(usingFloatingBase)
, nDoF(wbi->getDoFs())
{
    wbi_H_root = wbi::Frame();

    wbi_H_root_Vector = Eigen::VectorXd::Zero(16);
    wbi_T_root_Vector = Eigen::VectorXd::Zero(6);
}

IcubControllerServer::~IcubControllerServer()
{

}

std::shared_ptr<Model> IcubControllerServer::setRobotModel()
{
    return std::make_shared<OcraWbiModel>(robotName, wbi->getDoFs(), wbi, isFloatingBase);
}

void IcubControllerServer::getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root)
{
    q.resize(nDoF);
    qd.resize(nDoF);

    wbi->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data(), ALL_JOINTS);
    wbi->getEstimates(wbi::ESTIMATE_JOINT_VEL, qd.data(), ALL_JOINTS);

    if (isFloatingBase)
    {
        // Get root position as a 12x1 vector and get root vel as a 6x1 vector
        wbi->getEstimates(wbi::ESTIMATE_BASE_POS, wbi_H_root_Vector.data());
        wbi->getEstimates(wbi::ESTIMATE_BASE_VEL, wbi_T_root_Vector.data());

        // Convert to a wbi::Frame then to a Dispd
        wbi::frameFromSerialization(wbi_H_root_Vector.data(), wbi_H_root);
        OcraWbiConversions::wbiFrameToEigenDispd(wbi_H_root, H_root);

        // Fill the Twist.
        // Rotation then Translation
        T_root = Eigen::Twistd( wbi_T_root_Vector[3],
                                wbi_T_root_Vector[4],
                                wbi_T_root_Vector[5],
                                wbi_T_root_Vector[0],
                                wbi_T_root_Vector[1],
                                wbi_T_root_Vector[2]);
        // Eigen::Twistd T_root_wbi = Eigen::Twistd( wbi_T_root_Vector[0],
        //                         wbi_T_root_Vector[1],
        //                         wbi_T_root_Vector[2],
        //                         wbi_T_root_Vector[3],
        //                         wbi_T_root_Vector[4],
        //                         wbi_T_root_Vector[5]);
        // OcraWbiConversions::wbiToOcraTwistVector(T_root_wbi, T_root);


    }
}
