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
    iDynTree::JointPosDoubleArray qj;
    qj.resize(nDoF);

    wbi->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data(), ALL_JOINTS);
    wbi->getEstimates(wbi::ESTIMATE_JOINT_VEL, qd.data(), ALL_JOINTS);

    if (isFloatingBase)
    {
        if (useOdometry) {
            qj.zero();
            wbi->getEstimates(wbi::ESTIMATE_JOINT_POS, qj.data(), ALL_JOINTS);
//             std::cout << "\033[1;31m[DEBUG-ODOMETRY icubControllerServer::getRobotState]\033[0m Current joint configuration used to update kinematics is: " << qj.toString() << std::endl;
            
            // Fill wbi_H_root_Vector "manually" from odometry
            odometry.updateKinematics(qj);
            iDynTree::Transform wbi_H_root_Transform = odometry.getWorldLinkTransform(odometry.model().getDefaultBaseLink());
//             std::cout << "\033[1;31m[DEBUG-ODOMETRY icubControllerServer::getRobotState]\033[0m  wbi_H_rot_Transform: " << std::endl << wbi_H_root_Transform.toString() << std::endl;
            // This mapping is ROW-WISE
            Matrix<double, 16, 1> wbi_H_root_Vector_tmp(wbi_H_root_Transform.asHomogeneousTransform().data());
//             std::cout << "\033[1;31m[DEBUG-ODOMETRY icubControllerServer::getRobotState]\033[0m Rototrans from world to root is wbi_H_root_Vector: " << wbi_H_root_Vector_tmp << std::endl;
            wbi_H_root_Vector = wbi_H_root_Vector_tmp;
                      
            // For now assuming that contact of left and right foot is permanent for debugging purposes
            rootFrameVelocity(q, qd, wbi_H_root_Transform, 1.0e-6, 1, 1, wbi_T_root_Vector);
//             rootFrameVelocityPivLU(q, qd, wbi_H_root_Transform, 1.0e-6, 1, 1, wbi_T_root_Vector);
//             std::cout << "Root velocity is: " << wbi_T_root_Vector.transpose() << std::endl;
//            Eigen::MatrixXd jacobianRootLink = model->getSegmentJacobian("root_link");
//            wbi_T_root_Vector += jacobianRootLink*qd;
            
            
        } else {
            // Get root position as a 16x1 vector and get root vel as a 6x1 vector
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
    // The URDF file has mode joints than those used by the yarpWholeBodyInterface, and these two should match. Therefore, the following method creates a list of joints as those that constitute ROBOT_MAIN_JOINTS in yarpWholeBodyInterface.ini
    std::vector<std::string> consideredJoints = getCanonical_iCubJoints();
    if (!odometry.loadModelFromFileWithSpecifiedDOFs(model_file, consideredJoints)) {
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
        if (odometry.init(initialFixedFrame, "l_sole")) {
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

std::vector<std::string> IcubControllerServer::getCanonical_iCubJoints()
{
    std::vector<std::string> consideredJoints;
    
    // These are the joints that constitute ROBOT_MAIN_JOINTS and that will be hardcoded here just because the list name ROBOT_MAIN_JOINTS is hardcoded anyways. If that list was changed, then this must be changed too.
    consideredJoints.push_back("torso_pitch");
    consideredJoints.push_back("torso_roll");
    consideredJoints.push_back("torso_yaw");
    consideredJoints.push_back("l_shoulder_pitch");
    consideredJoints.push_back("l_shoulder_roll");
    consideredJoints.push_back("l_shoulder_yaw");
    consideredJoints.push_back("l_elbow");
    consideredJoints.push_back("l_wrist_prosup");
    consideredJoints.push_back("r_shoulder_pitch");
    consideredJoints.push_back("r_shoulder_roll");
    consideredJoints.push_back("r_shoulder_yaw");
    consideredJoints.push_back("r_elbow");
    consideredJoints.push_back("r_wrist_prosup");
    consideredJoints.push_back("l_hip_pitch");
    consideredJoints.push_back("l_hip_roll");
    consideredJoints.push_back("l_hip_yaw");
    consideredJoints.push_back("l_knee");
    consideredJoints.push_back("l_ankle_pitch");
    consideredJoints.push_back("l_ankle_roll");
    consideredJoints.push_back("r_hip_pitch");
    consideredJoints.push_back("r_hip_roll");
    consideredJoints.push_back("r_hip_yaw");
    consideredJoints.push_back("r_knee");
    consideredJoints.push_back("r_ankle_pitch");
    consideredJoints.push_back("r_ankle_roll");
    
    return consideredJoints;
}

void IcubControllerServer::rootFrameVelocity(Eigen::VectorXd& q,
                                             Eigen::VectorXd& qd,
                                             iDynTree::Transform& wbi_H_root_Transform,
                                             double           regularization,
                                             int              LEFT_FOOT_CONTACT,
                                             int              RIGHT_FOOT_CONTACT,
                                             Eigen::VectorXd& twist)
{
    wbi::Frame xBase(wbi_H_root_Transform.asHomogeneousTransform().data());
    int rootLinkID, leftFootID, rightFootID;
    wbi->getFrameList().idToIndex("root_link", rootLinkID);
    wbi->getFrameList().idToIndex("l_sole", leftFootID);
    wbi->getFrameList().idToIndex("r_sole", rightFootID);

    // Jacobian composed by the active constraints (for now left and right stacked! but THE LEFT OR RIGHT FOOT JACOBIANS SHOULD BE ZEROED WHEN THEY'RE DEACTIVATED. LEFT_FOOT_CONTACT and RIGHT_FOOT_CONTACT are binary variables (0 or 1) that indicate the activation of the contact.
    // Left foot jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> leftFootJacobian = Eigen::MatrixXd::Zero(6,nDoF+6);
    wbi->computeJacobian(q.data(), xBase, leftFootID, leftFootJacobian.data());
//     std::cout << "Left Foot Jacobian: " << std::endl;
//     std::cout << leftFootJacobian << std::endl;
    
    // Right foot jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> rightFootJacobian = Eigen::MatrixXd::Zero(6,nDoF+6);
    wbi->computeJacobian(q.data(), xBase, rightFootID, rightFootJacobian.data());
//     std::cout << "Right foot Jacobian: " << std::endl;
//     std::cout << rightFootJacobian << std::endl;
    
    // Concatenated jacobians
    Eigen::MatrixXd systemJacobian(leftFootJacobian.rows()+rightFootJacobian.rows(), leftFootJacobian.cols());
    systemJacobian.setZero();
    systemJacobian << leftFootJacobian, rightFootJacobian;
    // Concatenated jacobians for contacts contributions
//     std::cout << "Sytem Jacobian: " << std::endl;
//     std::cout << systemJacobian << std::endl;
    Eigen::MatrixXd contactsJacobianJointsContrib = systemJacobian.rightCols(nDoF);
    // 6x6 Jacobian including floating base part "Jacobian of the Floating Base Contribution"
//     Eigen::MatrixXd jacobianBaseContrib = model->getSegmentJacobian("root_link").leftCols(6);
    Eigen::MatrixXd jacobianBaseContrib = systemJacobian.leftCols(6);
//     std::cout << "Jacobian Base Contrib" << std::endl;
//     std::cout << jacobianBaseContrib << std::endl;
    // Pseudoinverse of the jacobian base contribution
    Eigen::MatrixXd pinvJacobianBaseContrib;
    pinv(jacobianBaseContrib, pinvJacobianBaseContrib, 0.01);
    // Floating-base velocity
    twist = -pinvJacobianBaseContrib * (contactsJacobianJointsContrib * qd);
    
}

void IcubControllerServer::rootFrameVelocityPivLU(Eigen::VectorXd& q,
                                                  Eigen::VectorXd& qd,
                                                  iDynTree::Transform& wbi_H_root_Transform,
                                                  double           regularization,
                                                  int              LEFT_FOOT_CONTACT,
                                                  int              RIGHT_FOOT_CONTACT,
                                                  Eigen::VectorXd& twist) 
{
    wbi::Frame xBase(wbi_H_root_Transform.asHomogeneousTransform().data());
    int rootLinkID, leftFootID, rightFootID;
    wbi->getFrameList().idToIndex("root_link", rootLinkID);
    wbi->getFrameList().idToIndex("l_sole", leftFootID);
    wbi->getFrameList().idToIndex("r_sole", rightFootID);
    
    // Left foot jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> leftFootJacobian = Eigen::MatrixXd::Zero(6,nDoF+6);
//     leftFootJacobian
    wbi->computeJacobian(q.data(), xBase, leftFootID, leftFootJacobian.data());
//     std::cout << "Left Foot Jacobian: " << std::endl;
//     std::cout << leftFootJacobian << std::endl;

    // Right foot jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> rightFootJacobian = Eigen::MatrixXd::Zero(6,nDoF+6);
    wbi->computeJacobian(q.data(), xBase, rightFootID, rightFootJacobian.data());
//     std::cout << rightFootJacobian << std::endl;
   
    
    // Using EIGEN. For now using lefFootJacobian (although it should the whole system's - concatenated jacobians)
    Eigen::PartialPivLU<Eigen::MatrixXd::PlainObject> luDecompositionBaseJacobian(6);
    luDecompositionBaseJacobian.compute(leftFootJacobian.leftCols<6>());
    twist = -luDecompositionBaseJacobian.solve(leftFootJacobian.rightCols(nDoF) * qd);
}

void IcubControllerServer::pinv(Eigen::MatrixXd mat, Eigen::MatrixXd& pinvmat, double pinvtoler) const
{
    Matrix<double,6,6> tmp = mat.transpose()*mat + pinvtoler*Eigen::Matrix<double,6,6>::Identity();
    pinvmat = tmp.inverse()*mat.transpose();
}


