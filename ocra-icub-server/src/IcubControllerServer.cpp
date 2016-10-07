#include <ocra-icub-server/IcubControllerServer.h>

// IcubControllerServer::IcubControllerServer()
// {
//
// }

IcubControllerServer::IcubControllerServer( std::shared_ptr<wbi::wholeBodyInterface> robot,
                                            std::string icubName,
                                            const bool usingFloatingBase,
                                            const ocra_recipes::CONTROLLER_TYPE ctrlType,
                                            const ocra_recipes::SOLVER_TYPE solver,
                                            const bool usingInterprocessCommunication,
                                            const bool useOdometry
                                        )
: ocra_recipes::ControllerServer(ctrlType, solver, usingInterprocessCommunication, useOdometry)
, wbi(robot)
, robotName(icubName)
, isFloatingBase(usingFloatingBase)
, useOdometry(useOdometry)
, nDoF(wbi->getDoFs())
{
    std::cout << "[DEBUG iCubControllerServer::icubControllerServer()  At construction time useOdometry is: " << this->useOdometry << std::endl;
    wbi_H_root = wbi::Frame();

    wbi_H_root_Vector = Eigen::VectorXd::Zero(16);
    wbi_T_root_Vector = Eigen::VectorXd::Zero(6);
}

IcubControllerServer::~IcubControllerServer()
{

}

ocra::Model::Ptr IcubControllerServer::loadRobotModel()
{
    return std::make_shared<ocra_icub::OcraWbiModel>(robotName, wbi->getDoFs(), wbi, isFloatingBase);
}

void IcubControllerServer::getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root)
{
    q.resize(nDoF);
    qd.resize(nDoF);

    wbi->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data(), ALL_JOINTS);
    wbi->getEstimates(wbi::ESTIMATE_JOINT_VEL, qd.data(), ALL_JOINTS);

    if (isFloatingBase)
    {
        if (useOdometry) {
            iDynTree::JointPosDoubleArray qj(nDoF);
            wbi->getEstimates(wbi::ESTIMATE_JOINT_POS, qj.data(), ALL_JOINTS);
            std::cout << "\033[1;31m[DEBUG-ODOMETRY icubControllerServer::getRobotState]\033[0m Current joint configuration used to update kinematics is: " << qj.toString() << std::endl;
            // Fill wbi_H_root_Vector "manually" from odometry
            odometry.updateKinematics(qj);
            iDynTree::Transform wbi_H_root_Transform = odometry.getWorldLinkTransform(odometry.model().getLinkIndex("root_link"));
//            Eigen::VectorXd wbi_H_root_Vector_tmp(wbi_H_root_Transform.asHomogeneousTransform().data());
            Matrix<double, 16, 1> wbi_H_root_Vector(wbi_H_root_Transform.asHomogeneousTransform().data());
            std::cout << "\033[1;31m[DEBUG-ODOMETRY icubControllerServer::getRobotState]\033[0m Rototrans from world to root is wbi_H_root_Vector: " << wbi_H_root_Vector << std::endl;
            // TODO: Velocity missing.
            // --------
            wbi->getEstimates(wbi::ESTIMATE_BASE_VEL, wbi_T_root_Vector.data());
            
        } else {
            // Get root position as a 12x1 vector and get root vel as a 6x1 vector
            wbi->getEstimates(wbi::ESTIMATE_BASE_POS, wbi_H_root_Vector.data());
            wbi->getEstimates(wbi::ESTIMATE_BASE_VEL, wbi_T_root_Vector.data());
        }
        
        // Convert to a wbi::Frame then to a Dispd
        wbi::frameFromSerialization(wbi_H_root_Vector.data(), wbi_H_root);
        ocra_icub::OcraWbiConversions::wbiFrameToEigenDispd(wbi_H_root, H_root);
        
        // Fill the Twist.
        // Rotation then Translation
        T_root = Eigen::Twistd( wbi_T_root_Vector[3],
                               wbi_T_root_Vector[4],
                               wbi_T_root_Vector[5],
                               wbi_T_root_Vector[0],
                               wbi_T_root_Vector[1],
                               wbi_T_root_Vector[2]);
    }
}

bool IcubControllerServer::initializeOdometry(std::string model_file, std::string initialFixedFrame)
{
    // TODO: Load URDF model found by the resource finder.
    if (!odometry.loadModelFromFile(model_file)) {
        std::cout << "[ERROR] icubcontrollerServer::initializeOdometry  Could not load URDF model of the robot from the specified path: " << model_file << std::endl;
        return false;
    }
    
    // Build a JointPosDoubleArray
    iDynTree::JointPosDoubleArray qj(wbi->getDoFs());
    qj.zero();
    wbi->getEstimates(wbi::ESTIMATE_JOINT_POS, qj.data());
    std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Current joint configuration for odometry is: " << qj.toString() << std::endl;
    std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m With " << wbi->getDoFs() << " DoF " << std::endl;
    
    // Initialize the odometry
    std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry's fixed link: "  << initialFixedFrame << std::endl;
    
    // Update kinematics
    if (odometry.updateKinematics(qj)) {
        std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry's kinematics was updated..." << std::endl;
        // By just specifying which is the initialFixedFrame, the default orientation of the world reference frame will be the same as the localWorldReferenceFrame found in yarpWholeBodyInterface.ini.
        if (odometry.init(initialFixedFrame)) {
            std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry was initialized... " << std::endl;
        } else {
            std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry could not be initialized!" << std::endl;
            return false;
        }
    } else {
        std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry could not be initialized due to a failed updateKinematics()" << std::endl;
        return false;
    }
    
    std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry was fully initialized!" << std::endl;
    return true;
}
