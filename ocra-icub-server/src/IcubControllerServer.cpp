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
    // OCRA_INFO("Getting robot state");
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

            // Fill wbi_H_root_Vector "manually" from odometry
            odometry.updateKinematics(qj);
            std::string currentFixedLink;
            controller->getFixedLinkForOdometry(currentFixedLink);
            odometry.changeFixedFrame(currentFixedLink);
            iDynTree::Transform wbi_H_root_Transform = odometry.getWorldLinkTransform(odometry.model().getDefaultBaseLink());
            // This mapping is ROW-WISE
            Matrix<double, 16, 1> wbi_H_root_Vector_tmp(wbi_H_root_Transform.asHomogeneousTransform().data());
            wbi_H_root_Vector = wbi_H_root_Vector_tmp;

            // Find out which tasks are active
            int leftSupport = 1; int rightSupport = 1;
            this->controller->getContactState(leftSupport, rightSupport);
            // For now assuming that contact of left and right foot is permanent for debugging purposes
            rootFrameVelocity(q, qd, wbi_H_root_Transform, 1e-5, leftSupport, rightSupport, wbi_T_root_Vector);
//             rootFrameVelocityPivLU(q, qd, wbi_H_root_Transform, wbi_T_root_Vector);
//             rootFrameVelocityPivLU(q, qd, wbi_H_root_Transform, leftSupport, rightSupport, wbi_T_root_Vector);

//             std::cout << "Root velocity is: " << wbi_T_root_Vector.transpose() << std::endl;


        } else {
            // Get root position as a 16x1 vector and get root vel as a 6x1 vector
            wbi->getEstimates(wbi::ESTIMATE_BASE_POS, wbi_H_root_Vector.data());
            wbi->getEstimates(wbi::ESTIMATE_BASE_VEL, wbi_T_root_Vector.data());
        }
//         qj.zero();

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
    this->controller->setFixedLinkForOdometry("l_sole");

    // Update kinematics
    if (odometry.updateKinematics(qj)) {
        std::cout << "\033[1;31m[DEBUG-ODOMETRY IcubControllerServer::initializeOdometry]\033[0m Odometry's kinematics was updated..." << std::endl;
        // By just specifying which is the initialFixedFrame, the default orientation of the world reference frame will be the same as the localWorldReferenceFrame found in yarpWholeBodyInterface.ini.
        // WARNING: What's stated in the previous line is actually not true, as I saw that if the second parameter is not given, some other orientation is used.
        std::string currentFixedLink;
        this->controller->getFixedLinkForOdometry(currentFixedLink);
        if (odometry.init(initialFixedFrame, currentFixedLink)) {
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
    consideredJoints.push_back("torso_pitch");          // index = 0
    consideredJoints.push_back("torso_roll");           // index = 1
    consideredJoints.push_back("torso_yaw");            // index = 2
    consideredJoints.push_back("l_shoulder_pitch");     // index = 3
    consideredJoints.push_back("l_shoulder_roll");      // index = 4
    consideredJoints.push_back("l_shoulder_yaw");       // index = 5
    consideredJoints.push_back("l_elbow");              // index = 6
    consideredJoints.push_back("l_wrist_prosup");       // index = 7
    consideredJoints.push_back("r_shoulder_pitch");     // index = 8
    consideredJoints.push_back("r_shoulder_roll");      // index = 9
    consideredJoints.push_back("r_shoulder_yaw");       // index = 10
    consideredJoints.push_back("r_elbow");              // index = 11
    consideredJoints.push_back("r_wrist_prosup");       // index = 12
    consideredJoints.push_back("l_hip_pitch");          // index = 13
    consideredJoints.push_back("l_hip_roll");           // index = 14
    consideredJoints.push_back("l_hip_yaw");            // index = 15
    consideredJoints.push_back("l_knee");               // index = 16
    consideredJoints.push_back("l_ankle_pitch");        // index = 17
    consideredJoints.push_back("l_ankle_roll");         // index = 18
    consideredJoints.push_back("r_hip_pitch");          // index = 19
    consideredJoints.push_back("r_hip_roll");           // index = 20
    consideredJoints.push_back("r_hip_yaw");            // index = 21
    consideredJoints.push_back("r_knee");               // index = 22
    consideredJoints.push_back("r_ankle_pitch");        // index = 23
    consideredJoints.push_back("r_ankle_roll");         // index = 24

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

    // Right foot jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> rightFootJacobian = Eigen::MatrixXd::Zero(6,nDoF+6);
    wbi->computeJacobian(q.data(), xBase, rightFootID, rightFootJacobian.data());

    // Concatenated jacobians
    Eigen::MatrixXd systemJacobian(leftFootJacobian.rows()+rightFootJacobian.rows(), leftFootJacobian.cols());
    systemJacobian.setZero();
    systemJacobian << LEFT_FOOT_CONTACT*leftFootJacobian, RIGHT_FOOT_CONTACT*rightFootJacobian;
    // Concatenated jacobians for contacts contributions
    Eigen::MatrixXd contactsJacobianJointsContrib = systemJacobian.rightCols(nDoF);
    // 6x6 Jacobian including floating base part "Jacobian of the Floating Base Contribution"
//     Eigen::MatrixXd jacobianBaseContrib = model->getSegmentJacobian("root_link").leftCols(6);
    Eigen::MatrixXd jacobianBaseContrib = systemJacobian.leftCols(6);
    // Pseudoinverse of the jacobian base contribution
    Eigen::MatrixXd pinvJacobianBaseContrib;
    pinv(jacobianBaseContrib, pinvJacobianBaseContrib, regularization);
    // Floating-base velocity
    twist = -pinvJacobianBaseContrib * (contactsJacobianJointsContrib * qd);

}

void IcubControllerServer::pinv(Eigen::MatrixXd mat, Eigen::MatrixXd& pinvmat, double pinvtoler) const
{
    Matrix<double,6,6> tmp = mat.transpose()*mat + pinvtoler*Eigen::Matrix<double,6,6>::Identity();
    pinvmat = tmp.inverse()*mat.transpose();
}

void IcubControllerServer::rootFrameVelocityPivLU(Eigen::VectorXd& q,
                                                  Eigen::VectorXd& qd,
                                                  iDynTree::Transform& wbi_H_root_Transform,
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

    Eigen::PartialPivLU<Eigen::MatrixXd::PlainObject> luDecompositionBaseJacobian(6);
    luDecompositionBaseJacobian.compute(leftFootJacobian.leftCols<6>());
    twist = -luDecompositionBaseJacobian.solve(leftFootJacobian.rightCols(nDoF) * qd);
}

void IcubControllerServer::rootFrameVelocityPivLU(Eigen::VectorXd& q,
                                                  Eigen::VectorXd& qd,
                                                  iDynTree::Transform& wbi_H_root_Transform,
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
    wbi->computeJacobian(q.data(), xBase, leftFootID, leftFootJacobian.data());
//     OCRA_WARNING("Left foot Jacobian: " << std::endl << leftFootJacobian);

    // Right foot jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> rightFootJacobian = Eigen::MatrixXd::Zero(6,nDoF+6);
    wbi->computeJacobian(q.data(), xBase, rightFootID, rightFootJacobian.data());
//     OCRA_WARNING("Right foot Jacobian: " << std::endl << rightFootJacobian);

    // System Jacobian
    Eigen::MatrixXd systemJacobian(leftFootJacobian.rows()+rightFootJacobian.rows(), leftFootJacobian.cols());
    systemJacobian.setZero();
    systemJacobian.block(0,0,6,nDoF+6) = LEFT_FOOT_CONTACT*leftFootJacobian;
    systemJacobian.block(6,0,6,nDoF+6) = RIGHT_FOOT_CONTACT*rightFootJacobian;
//     systemJacobian << LEFT_FOOTrootFrameVelocityPivLU_CONTACT*leftFootJacobian, RIGHT_FOOT_CONTACT*rightFootJacobian;
//     OCRA_WARNING("System Jacobian is: " << std::endl << systemJacobian << std::endl);
//     OCRA_WARNING("DOF: " << nDoF << " LEFT_FOOT_CONTACT: " << LEFT_FOOT_CONTACT << " RIGHT_FOOT_CONTACT: " << RIGHT_FOOT_CONTACT << std::endl);

    Eigen::PartialPivLU<Eigen::MatrixXd::PlainObject> luDecompositionBaseJacobian(6);
    luDecompositionBaseJacobian.compute(leftFootJacobian.leftCols<6>());
    twist = -luDecompositionBaseJacobian.solve(leftFootJacobian.rightCols(nDoF) * qd);

    Eigen::MatrixXd B = leftFootJacobian.rightCols(nDoF) * qd;
//     velocityError(systemJacobian.leftCols(6), B, twist);
}

void IcubControllerServer::velocityError(MatrixXd A, MatrixXd B, MatrixXd X) {
    double error = (A*X - B).norm()/B.norm();
    if (error > 1e-3)
        OCRA_WARNING("Floating-base velocity error too big!: " << error);
}
