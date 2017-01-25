/**
 *  \class ZmpPreviewController
 *
 *  \brief Implementes an extended ZMP preview controller as an unconstrained QP problem.
 *
 *  \note Put original ZMP preview control reference along with Aurelien's
 *
 *  \author Jorhabib Eljaik
 *
 *  \cite ibanezThesis2015
 *
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *
 *  \details Given a rough ZMP trajectory and a corresponding consistent CoM horizontal velocity, this class computes at a fast rate optimal CoM jerks over a refined preview horizon. The ZMP preview control formulation in (reference goes here) is extended to account for a CoM velocity tracking objective in order to minimize the error over a preview horizon to reference CoM velocities. This preview controller solves the following problem in a time window of size \f$N_c\f$

 \f{align*}
 \mathcal{U}_{k+N_c|k}^* = \underset{\mathcal{U}_{k+N_c|k}}{\text{arg min}} \; \sum_{j=1}^{N} & \eta_b || \mathbf{p}_{k+j|k} - \mathbf{r}_{k+j|k}^r ||^2 + \eta_w ||\mathbf{\dot{h}}_{k+j|k} - \dot{\mathbf{h}}^r_{k+j|k} ||^2 + \eta_u || \mathbf{u} ||^2 \\
 \text{such that}&\\
 \mathbf{p} &= \mathbf{h} - \mathbf{c}\mathbf{e}_2 \mathbf{\ddot{h}} \\
 \mathbf{\hat{h}}_{k+j+1|k} &= \mathbf{A}_h \hat{\mathbf{h}}_{k+j|k} + \mathbf{B}_h \mathbf{u}_{k+j+1|k} \\
 \mathbf{h}_{k+j|k} &= \mathbf{h}_k\\
 \mathbf{h} &= \mathbf{c} - (\mathbf{c}\cdot\mathbf{e}_2)\mathbf{e}_2
 \f}

 Where \f${U}_{k+N_c|k}^*\f$ is the optimal horizon of inputs, \f$\mathbf{p}\f$ is the horizontal ZMP coordinate, \f$ \mathbf{r}^r \f$ ZMP references (in `walking-client` it's the interpolated ZMP reference from the walking MIQP problem) \f$ \mathbf{\dot{h}} \f$ the horizontal CoM velocity, \f$ \mathbf{\dot{h}}^r \f$ CoM horizontal velocity reference (in walking-client it's the interpolated CoM velocity reference from the walking MIQP problem). \f$\mathbf{u}\f$ is the input CoM jerk, \f$\mathbf{c}\f$ the three-dimensional CoM position. The horizontal CoM state is denoted \f$\hat{\mathbf{h}} = (\mathbf{h}, \mathbf{\dot{h}}, \mathbf{\ddot{h}}) \f$  (CoM horizontal position, velocity and acceleration respectively) while \f$\mathbf{e}_2\f$ is the vertical unit vector \f$ [0\;0\;1]^T\f$. \f$\mathbf{A}_h\f$ and \f$\mathbf{B}_h\f$ are integration matrices.

 The previously described problem is an unconstrained QP problem which has a closed-form solution. The same can be expressed in matrix form as:

 \f[
 \underset{\mathbf{U}}{\text{min}} \; (\mathbf{P}_r - \mathbf{P})^T \mathbf{N}_b (\mathbf{P}_r - \mathbf{P}) + (\tilde{\mathbf{H}}_r - \mathbf{H})^T\mathbf{N}_w(\tilde{\mathbf{H}}_r - \mathbf{H}) + \mathbf{U}^T\mathbf{N}_u \mathbf{U}
 \f]

 Where:
 \f{align*}
 \mathbf{U}^T &= \left[ \mathbf{u}^T_{k+1|k}, \cdots, \mathbf{u}^T_{k+N_c|k} \right]^T \\
 \mathbf{P}^T &= \left[ \mathbf{p}^T_{k+1|k}, \cdots, \mathbf{p}^T_{k+N_p|k} \right]^T \\
 \mathbf{P}^T_r &= \left[ \mathbf{r}^T_{k+1|k}, \cdots, \mathbf{r}^T_{k+N_p|k} \right]^T \\
 \mathbf{\tilde{H}}^T &= \left[ \dot{\mathbf{h}}^T_{k+1|k}, \cdots, \dot{\mathbf{h}}^T_{k+N_p|k} \right]^T \\
 \mathbf{\tilde{H}}^T_r &= \left[ \dot{\mathbf{h}^r}^T_{k+1|k}, \cdots, \dot{\mathbf{h}^r}^T_{k+N_p|k} \right]^T \\
 \mathbf{N}_b &= \eta_b\left[\begin{array}{ccc}
 1 & \cdots & 0 \\
 \vdots & \ddots & \vdots\\
 0  & \cdots & 1
 \end{array}\right]
 \f}

 \f$\mathbf{N}_w\f$ and \f$\mathbf{N}_u\f$ have a similar structure to \f$\mathbf{N}_b\f$.

 A closed-form solution can thus be found and is equal to:
 \f[
 \mathbf{U}_{k+N_c|k} = (\mathbf{H}_p^T \mathbf{N}_b \mathbf{H}_p + \mathbf{N}_u + \mathbf{H}_h^T \mathbf{N}_w \mathbf{H}_h)^{-1} \left(\mathbf{H}^T_p \mathbf{N}_b (\mathbf{P}_r - \mathbf{G}_p \hat{\mathbf{h}}_k) + \mathbf{H}^T_h\mathbf{N}_w(\tilde{\mathbf{H}}_r - \mathbf{G}_h \hat{\mathbf{h}}_k)\right)
 \f]


 */

#ifndef _ZMPPREVIEWCONTROLLER_
#define _ZMPPREVIEWCONTROLLER_

#include <ocra-icub/Utilities.h>
#include <ocra/util/ErrorsHelper.h>
#include <Eigen/Dense>
#include <vector>
#include "walking-client/utils.h"


struct ZmpPreviewParams {
    /**
     *  Size of the control window.
     */
    int Nc;
    /** 
     * Size of the preview window.
     */
    int Np;
    /**
     *  Robot's CoMconstant height.
     */
    double cz;
    /**
     *  Weight of the input regularization term in the cost function \f$\eta_u\f$.
     */
    double nu;
    /**
     *  Weight of the walking cost function \f$ \eta_w \f$.
     */
    double nw;
    /**
     *  Weight of the balancing cost function \f$ \eta_b \f$.
     */
    double nb;
//     std::vector<Eigen::Vector2d> Pr;
//     std::vector<Eigen::Vector2d> Hr;

    /**
     * Constructor
     */
    ZmpPreviewParams(int    Np,
                     int    Nc,
                     double cz,
                     double nu,
                     double nw,
                     double nb):
    Np(Np),
    Nc(Nc),
    cz(cz),
    nu(nu),
    nw(nw),
    nb(nb){}

};

class ZmpPreviewController
{
public:
    /**
     *  Class constructor. Builds all the time-invariant matrices used to compute the optimal output of the preview controller.
         @param period       Period (in ms) of the thread in which an object of this class is created.
         @param parameters   An object containing the main parameters. \see struct ZmpPreviewParams for details on the parameters.
     */
    ZmpPreviewController(const double period, std::shared_ptr<ZmpPreviewParams> parameters);

    virtual ~ZmpPreviewController();


    /**
     Initializes the matrices and parameters used by the controller.

     Currently we are not initializing through this method, but through the constructor.
     @return True if successful, false otherwise.
     */
    bool initialize();

    /**
     *  Computes a horizon of optimal inputs (CoM jerks) to be applied to the system, i.e.:
     *  \f{align*}
         \mathcal{U}_{k+N|k} &= (\mathbf{H}_p^T \mathbf{N}_b \mathbf{H}_p + \mathbf{N}_u + \mathbf{H}_h^T \mathbf{N}_w \mathbf{H}_h)^{-1} \left(\mathbf{H}^T_p \mathbf{N}_b (\mathbf{P}_r - \mathbf{G}_p \hat{\mathbf{h}}_k) + \mathbf{H}^T_h\mathbf{N}_w(\tilde{\mathbf{H}}_r - \mathbf{G}_h \hat{\mathbf{h}}_k)\right)\\
         \mathcal{U}_{k+N|k} &= \mathbf{A}_{\text{opt}}^{-1} \mathbf{b}_{\text{opt}}
         \f}
     *
     *  This solution is computed through standard Cholesky decomposition. See Eigen::LLT for more details. Matrix ZmpPreviewController::AOptimal is precomputed by the constructor for efficiency reasons.
     *
     *  @param zmpRef    \f$\mathbf{P}_r\f$ Horizon of \f$N_c\f$ ZMP references.
     *  @param comVelRef \f$\tilde{\mathbf{H}}_r\f$ Horizon of \f$N_c\f$ CoM velocities.
     *  @param hk        Current measured CoM state at time \f$k\f$, i.e. \f$\hat{\mathbf{h}}_k\f$
     *  @param optimalU  Closed-form solution to the unconstrained QP problem, \f$\mathcal{U}_{k+N_c|k}\f$
     *  @see ZmpPreviewController::AOptimal, ZmpPreviewController::bOptimal
     *
     */
    template <typename Derived>
    void computeOptimalInput(const Eigen::MatrixBase<Derived>& zmpRef,
                             const Eigen::MatrixBase<Derived>& comVelRef,
                             const Eigen::MatrixBase<Derived>& hk,
                             Eigen::MatrixBase<Derived>& optimalU) {
//     AOptimal = Hp.transpose()*Nb*Hp + Nu + Hh.transpose()*Nw*Hh;
//     bOptimal = Hp.transpose()*Nb*(zmpRef - Gp*hk) + Hh.transpose()*Nw*(comVelRef - Gh*hk);
//     optimalU = AOptimal.colPivHouseholderQr().solve(bOptimal);

//         double start = yarp::os::Time::now();
        //TODO: Remember that the bOptimal I have to use is actually the next line, not Weiber's because it doesn't take into account com velocity references.
//         bOptimal = Hp.transpose()*Nb*(zmpRef - Gp*hk) + Hh.transpose()*Nw*(comVelRef - Gh*hk);
//         OCRA_INFO("Time to compute bOptimal is: " << yarp::os::Time::now() - start);
        bOptimal = Hp.transpose() * (zmpRef - Gp * hk);
//         start = yarp::os::Time::now();
//         optimalU = AOptimal.colPivHouseholderQr().solve(Hp.transpose() * (bOptimal));
        // Using Cholesky decomposition
        optimalU = (AOptimal).llt().solve(bOptimal);
//         OCRA_INFO("Time to compute optimalU using Cholesky decomposition : " << yarp::os::Time::now() - start);
    }

    /**
     *  Integrates the CoM for a given CoM jerk input. Integration matrices \f$\mathbf{A}_h\f$ and \f$\mathbf{b}_h\f$ are precomputed by the constructor of this class.
     *
     *  \f[
     *  \mathbf{h}_{k+1} = \mathbf{A}_h \mathbf{h}_k + \mathbf{B}_h \mathbf{u}_k
     *  \f]
     *
     *  @param      comJerk  Input com jerk \f$\mathbf{u}_k\f$
     *  @param      hk       Current COM state \f$\mathbf{h}_k\f$
     *  @param[out] hkk      Integrated COM state \f$\mathbf{h}_{k+1}\f$
     *  @see                 buildAh(), buildBh()
     */
    void integrateCom(Eigen::VectorXd comJerk, Eigen::VectorXd hk, Eigen::VectorXd &hkk);


    /** Computes a simplified model-based ZMP.

     Computes the zero-moment point given the robot's horizontal CoM acceleration and position, besides
     a constant CoM height and gravity acceleration.

     @param      hk Horizontal CoM position.
     @param      ddhk Horizontal CoM acceleration.
     @param[out] p Computed ZMP.
     @note       The MPC formulation presented in the description of this controller finds the optimal
                 inputs (jerks) for the system, not the optimal ZMP trajectory! This is why, if we want
                 to see what the previewed ZMP reference is, we need to use this table-cart model whose
                 inputs are to be integrated from the optimal jerks through integrateCom().
     */
    void tableCartModel(Eigen::Vector2d hk, Eigen::VectorXd ddhk, Eigen::Vector2d& p);

     /** Computes a simplified model-based ZMP.

     Computes the zero-moment point given the robot's horizontal CoM acceleration and position, besides
     a constant CoM height and gravity acceleration.
     
     \f[
     \mathbf{p}_{k+1} = \mathbf{C}_p \mathbf{h}_{k+1}
     \f]

     @param      hkk Horizontal CoM state.
     @param[out] p Computed ZMP.
     @note       The MPC formulation presented in the description of this controller finds the optimal
                 inputs (jerks) for the system, not the optimal ZMP trajectory! This is why, if we want
                 to see what the previewed ZMP reference is, we need to use this table-cart model whose
                 inputs are to be integrated from the optimal jerks through integrateCom().
     */
    void tableCartModel(Eigen::VectorXd hkk, Eigen::Vector2d& p);

    /**
     *  Builds \f$A_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Ah.
     *  @see ZmpPreviewController::Ah
     */
    Eigen::MatrixXd buildAh(const double dt);

    /**
     *  Builds \f$B_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Bh.
     *  @see ZmpPreviewController::Bh
     */
    Eigen::MatrixXd buildBh(const double dt);

    /**
     *  Builds \f$C_p\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param cz Constant CoMheight.
     *  @param g  Constant gravity acceleration (m^2/s)
     *
     *  @return \f$C_p\f$
     *  @see ZmpPreviewController::Cp
     */
    Eigen::MatrixXd buildCp(const double cz, const double g);

    /**
     *  Builds \f$G_p\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param Cp \f$C_p\f$
     *  @param Ah \f$A_h\f$
     *  @param Nc \f$N_p\f$
     *
     *  @return \f$G_p\f$
     *  @see ZmpPreviewController::Gp
     */
    Eigen::MatrixXd buildGp(Eigen::MatrixXd Cp, Eigen::MatrixXd Ah, const int Np);

    /**
     *  Builds \f$H_p\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param Cp \f$C_p\f$
     *  @param Bh \f$B_h\f$
     *  @param Ah \f$A_h\f$
     *  @param Nc \f$N_c\f$
     *  @param Np \f$N_p\f$
     *
     *  @return \f$H_p\f$
     *  @see ZmpPreviewController::Hp
     */
    Eigen::MatrixXd buildHp(Eigen::MatrixXd Cp, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc, const int Np);

    /**
     *  Builds \f$C_h\f$. Called during the member list initialization of the constructor of this class
     *
     *  @return \f$C_h\f$
     *  @see ZmpPreviewController::Ch
     */
    Eigen::MatrixXd buildCh();

    /**
     *  Builds \f$G_h\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param Ch \f$C_h\f$
     *  @param Ah \f$A_h\f$
     *  @param Np \f$N_p\f$
     *
     *  @return \f$G_h\f$
     *  @see ZmpPreviewController::Gh
     */
    Eigen::MatrixXd buildGh(Eigen::MatrixXd Ch, Eigen::MatrixXd Ah, const int Np);

    /**
     *  Builds \f$H_h\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param Ch \f$C_h\f$
     *  @param Bh \f$B_h\f$
     *  @param Ah \f$A_h\f$
     *  @param Hh \f$N_c\f$
     *  @param Nc \f$N_p\f$
     *
     *  @return \f$H_h\f$
     *  @see ZmpPreviewController::Hh
     */
    Eigen::MatrixXd buildHh(Eigen::MatrixXd Ch, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc, const int Np);

    /**
     *  Builds \f$N_u\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param nu \f$\eta_u\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$N_u\f$
     *  @see ZmpPreviewController::Nu
     */
    Eigen::MatrixXd buildNu(const double nu, const int Nc);

    /**
     *  Builds \f$N_w\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param nw \f$\eta_w\f$
     *  @param Np \f$Nc\f$
     *
     *  @return \f$N_w\f$
     *  @see ZmpPreviewController::Nw
     */
    Eigen::MatrixXd buildNw(const double nw, const int Np);

    /**
     *  Builds \f$N_b\f$. Called during the member list initialization of the constructor of this class
     *
     *  @param nb \f$\eta_b\f$
     *  @param Np \f$N_p\f$
     *
     *  @return \f$N_b\f$
     *  @see ZmpPreviewController::Nb
     */
    Eigen::MatrixXd buildNb(const double nb, const int Np);

    /**
     *  Computes the ZMP for a single foot in world reference frame.
     *
     *  Assuming that \f$\mathbf{p}\f$ is the position of the ZMP for a single foot, \f$\mathbf{p}_s\f$ the position of a force torque (F/T) sensor at the foot, the ZMP position can be computed as:
     \f[
     \left[\begin{array}{c}p_x \\
     p_y \end{array}\right] = \frac{1}{f_z}
     \left[\begin{array}{cccccc}
     -p_{s_z} & 0 & p_{s_x} & 0 & -1 & 0 \\
     0 & -p_{s_z} & p_{s_y} & 1 & 0 & 0
     \end{array}\right]
     \left[\begin{array}{c}
     \mathbf{f}\\
     \mathbf{\tau}
     \end{array}\right]
     \f]

     *
     *  @param whichFoot        LEFT_FOOT or RIGHT_FOOT.
     *  @param wrench           External wrench on the foot as read by the F/T sensors.
     *  @param[out] footZMP     Foot ZMP in world reference frame.
     *  @param[out] wrenchInWorldRef Transformed wrench in world reference frame.
     *  @param tolerance        Tolerance value below which the ZMP is considered null.
     *  @cite                   Kajita2014Intro
     *
     *  @return True if all operations proceed successfully.
     */
    bool computeFootZMP(FOOT whichFoot,
                        Eigen::VectorXd wrench,
                        Eigen::Vector2d &footZMP,
                        Eigen::VectorXd &wrenchInWorldRef,
                        ocra::Model::Ptr model,
                        const double tolerance=1e-3);


    /**
     *  Computes the global ZMP for two feet in contact.
     *
     *  After obtaining the ZMP position for both feet \f$\mathbf{p}_R\f$ and \f$\mathbf{p}_L\f$ independently and expressed in the world reference frame, in the case where both feet are in contact with the ground (or just one), the global expression of the ZMP \f$\mathbf{p}\f$ expressed in the world reference frame is:
     \f[
     \left[\begin{array}{c}
     p_x\\
     p_y
     \end{array}\right] =
     \frac{1}{f_{R_z} + f_{L_z}}
     \left[\begin{array}{cc}
     \mathbf{p}_R & \mathbf{p}_L
     \end{array}\right]
     \left[\begin{array}{c}
     f_{R_z}\\
     f_{L_z}
     \end{array}\right]
     \f]
     *
     *  @param rawLeftFootWrench  Raw left foot wrench as read from the sensors [force | torque]
     *  @param rawRightFootWrench Raw right foot wrench as read from the sensors.
     *  @param globalZMP          Global zmp in world reference frame considering both feet.
     @  @cite                     Kajita2014Intro
     *
     *  @return True if all operations succeed, false otherwise.
     */
    bool computeGlobalZMPFromSensors(Eigen::VectorXd rawLeftFootWrench,
                                     Eigen::VectorXd rawRightFootWrench,
                                     ocra::Model::Ptr model,
                                     Eigen::Vector2d &globalZMP);

    /**
     *  Retrieves the FT sensor adjoint matrix expressed in the world reference frame which multiplied by the local measurement of the sensor gives you the measurement in the world reference.
     *
     *  @param      whichFoot        LEFT_FOOT or RIGHT_FOOT
     *  @param[out] T                Adjoint matrix.
     *  @param[out] sensorPosition   3D Position of the F/T sensor corresponding to whichFoot. 
     *  @param      model            Pointer to the model that must have been initialized by the calling thread.
     */
    void getFTSensorAdjointMatrix(FOOT whichFoot, Eigen::MatrixXd &T, Eigen::Vector3d &sensorPosition, ocra::Model::Ptr model);


private:
    /**
     *  CoMconstant height. It is not hardcoded but corresponds to the vertical coordinate (height) of the CoMat the beginning of execution, i.e. \f$ c_z \f$.
     */
    const double cz;
    /**
     *  Gravity constant value in m/s^2, \f$ g \f$.
     */
    const double g = 9.8;
    /**
     *  Rate of the thread where this ZmpPreviewController is executed.
     */
    const double dt;
    /**
     *  Size of control window \f$ N_c \f$. It should less or equal to \f$Np\f$.
     */
    const int Nc;
    /**
     *  Size of preview window \f$ N_p \f$. It should be greater or equal to \f$N_c\f$.
     */
    const int Np;
    /**
     *  Weight of the input regularization term in the cost function \f$\eta_u\f$
     */
    const double nu;
    /**
     *  Weight of the walking cost function \f$ \eta_w \f$
     */
    const double nw;
    /**
     *  Weight of the balancing cost function \f$ \eta_b \f$
     */
    const double nb;
    /**
     *  Diagonal matrix \f$N_u = \eta_u\mathbf{I}_{2N_c}\f$ of size \f$[2N_c \times 2N_c]\f$ weighting the input regularization term in the matricial expression of the controller's cost function.
     */
    const Eigen::MatrixXd Nu;
    /**
     *  Diagonal matrix \f$N_w\ = \eta_w\mathbf{I}_{2N_p}\f$ of size \f$[2N_p \times 2N_p]\f$ weighting the walking cost function in its matrix form.
     */
    const Eigen::MatrixXd Nw;
    /**
     *  Diagonal matrix \f$N_b = \eta_b\mathbf{I}_{2N_p}\f$ of size \f$[2N_p \times 2N_p]\f$ weighting the balancing cost function in its matrix form.
     */
    const Eigen::MatrixXd Nb;
    /**
     *  State matrix \f$\mathbf{A}_h\f$ from the linear state process of the CoMstate \f$\hat{\mathbf{h}}\f$. It is a constant matrix of size \f$6\times6\f$ equal to:
     \f[
     \mathbf{A_h} = \left[ \begin{array}{ccc}
     \mathbf{I}_2  & \delta t \mathbf{I}_2  &  \frac{\delta t^2}{2} \mathbf{I}_2 \\
     0  &     \mathbf{I}_2     &  \delta t     \\
     0  &     0     &      \mathbf{I}_2
     \end{array} \right]
     \f]
     */
    const Eigen::MatrixXd Ah;
    /**
     *  Input matrix \f$\mathbf{B}_h\f$ from the linear state process of the CoMstate \f$\hat{\mathbf{h}}\f$. It is constant of size \f$6\times2\f$ and equal to:
     \f[
     \mathbf{B_h} = \left[ \begin{array}{c}
     \frac{\delta^3}{6}\mathbf{I}_2 \\
     \frac{\delta t^2}{2} \mathbf{I}_2     \\
     \delta t \mathbf{I}_2
     \end{array} \right]
     \f]
     */
    const Eigen::MatrixXd Bh;
    /**
     *  Output matrix \f$C_p\f$ from the linear state process relating ZMP to the CoMdynamics \f$\hat{\mathbf{h}}\f$. It is time-invariant of size \f$2\times6\f$ and equal to:
     \f[
     \mathbf{C}_p = \left[\begin{array}{ccc}
     \mathbf{I}_2  &  \mathbf{0}_2  &   -\frac{c_z}{g}\mathbf{I}_2 \\
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Cp;
    /**
     *  State matrix \f$\mathbf{G}_p\f$ from the preview horizon of ZMP outputs \f$\mathbf{P}\f$. It is of size \f$2N_p \times 6\f$ and equal to:
     \f[
     \mathbf{G}_p = \left[\begin{array}{c}
     \mathbf{C}_p\mathbf{A}_h \\
     \vdots \\
     \mathbf{C}_p\mathbf{A}^{N_p}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Gp;
    /**
     *  Input matrix \f$\mathbf{H}_p\f$ from the preview horizon of ZMP outputs \f$\mathbf{P}\f$. It is of size \f$2N_p \times 2N_c\f$ and equal to:
     \f[
     \mathbf{H}_p = \left[\begin{array}{cccc}
     \mathbf{C}_p\mathbf{B}_h               &   0                          &  \cdots   &   0 \\
     \mathbf{C}_p\mathbf{A}_h\mathbf{B}_h   &   \mathbf{C}_p\mathbf{B}_h   &  \cdots   &   0 \\
     \vdots                                 & \vdots                       & \ddots    &  \vdots \\
     \mathbf{C}_p\mathbf{A}^{N_p-1}_h\mathbf{B}_h & \mathbf{C}_p\mathbf{A}^{N_p-2}_h\mathbf{B}_h & \cdots & \mathbf{C}_p\mathbf{A}_p^{N_p - N_c}\mathbf{B}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Hp;
    /**
     *  Output matrix of the CoMlinear process state transition where the CoMhorizontal velocity is the output. It is of size \f$2\times6\f$ and equal to:
     \f[
     \mathbf{C}_h = \left[\begin{array}{ccc}
     \mathbf{0}_2  &  \mathbf{I}_2  &   \mathbf{0}_2 \\
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Ch;
    /**
     *  State matrix \f$\mathbf{G}_h\f$  of size \f$2N_p \times 6\f$ from the preview horizon of CoMvelocities. It is constant and equal to:
     \f[
     \mathbf{G}_h = \left[\begin{array}{c}
     \mathbf{C}_h\mathbf{A}_h \\
     \vdots \\
     \mathbf{C}_h\mathbf{A}^{N_p}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Gh;
    /**
     *  Input matrix \f$\mathbf{H}_h\f$ from the preview horizon of CoMvelocities \f$\tilde{\mathbf{H}}\f$, of size \f$2N_p \times 2N_c\f$ and equal to:
     \f[
     \mathbf{H}_h = \left[\begin{array}{cccc}
     \mathbf{C}_h\mathbf{B}_h               &   0                          &  \cdots   &   0 \\
     \mathbf{C}_h\mathbf{A}_h\mathbf{B}_h   &   \mathbf{C}_h\mathbf{B}_h   &  \cdots   &   0 \\
     \vdots                                 & \vdots                       & \ddots    &  \vdots \\
     \mathbf{C}_h\mathbf{A}^{N_p-1}_h\mathbf{B}_h & \mathbf{C}_h\mathbf{A}^{N_p-2}_h\mathbf{B}_h & \cdots & \mathbf{C}_h\mathbf{A}^{N_p - N_c}\mathbf{B}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Hh;

    /**
     *  Matrix \f$\mathbf{A}_{\text{opt}}\f$ in computeOptimalInput().
     */
    Eigen::MatrixXd AOptimal;

    /**
     *  Vector \f$\mathbf{b}_{\text{opt}}\f$ in computeOptimalInput().
     */
    Eigen::MatrixXd bOptimal;



};

#endif
