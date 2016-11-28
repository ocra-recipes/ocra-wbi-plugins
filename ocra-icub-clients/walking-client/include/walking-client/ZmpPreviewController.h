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
 *  \details Given a rough ZMP trajectory and a corresponding consistent CoM velocity, this class computes at a fast rate optimal CoM jerks over a refined preview horizon. The ZMP preview control formulation in (reference goes here) is extended to account for a CoM tracking objective in order to minimize the error over a preview horizon to reference CoM velocities. This preview controller solves with
 
 \f[
 \mathcal{U}_{k+N|k} = (\mathbf{u}_{k|k}, ... , \mathbf{u})
 \f]
 
 Denoting a horizon of input CoM jerks \f$\mathbf{u}\f$, the following problem:
 
 \f{align*}
 \underset{\mathcal{U}_{k+N|k}}{\text{min}} \; \sum_{j=1}^{N} & \eta_b || \mathbf{p}_{k+j|k} - \mathbf{r}_{k+j|k}^r ||^2 + \eta_w ||\mathbf{\dot{h}}_{k+j|k} - \dot{\mathbf{h}}^r_{k+j|k} ||^2 + \eta_r || \mathbf{u} ||^2 \\
 \text{such that}&\\
 \mathbf{p} &= \mathbf{h} - \mathbf{c}\mathbf{e}_2 \mathbf{\ddot{h}} \\
 \mathbf{\hat{h}}_{k+j+1|k} &= \mathbf{A}_h \hat{\mathbf{h}}_{k+j|k} + \mathbf{B}_h \mathbf{u}_{k+j+1|k} \\
 \mathbf{h}_{k+j|k} &= \mathbf{h}_k\\
 \mathbf{h} &= \mathbf{c} - (\mathbf{c}\cdot\mathbf{e}_2)\mathbf{e}_2
 \f}
 
 Where \f$\mathbf{p}\f$ are horizontal ZMP coordinates, \f$ \mathbf{r}^r \f$ ZMP references (in walking-client is the interpolated ZMP reference from the walking MIQP problem) \f$ \mathbf{\dot{h}} \f$ the horizontal COM velocity, \f$ \mathbf{\dot{h}}^r \f$ COM horizontal velocity reference (in walking-client it's the interpolated COM velocity reference from the walking MIQP problem). \f$\mathbf{u}\f$ are the input COM jerks, \f$h\f$ the horizonal COM position, \f$\hat{h}\f$ the horizontal COM state \f$ (\mathbf{h}, \mathbf{\dot{h}}, \mathbf{\ddot{h}}) \f$ and \f$\mathbf{e}_2\f$ the vertical unit vectory \f$ [0\;0\;1]^T\f$. \f$\mathbf{A}_h\f$ and \f$\mathbf{B}_h\f$ are integration matrices.
 */

#ifndef _ZMPPREVIEWCONTROLLER_
#define _ZMPPREVIEWCONTROLLER_

#include <ocra-icub/Utilities.h>
#include <ocra/util/ErrorsHelper.h>
#include <Eigen/Dense>
#include <vector>


class ZmpPreviewController
{
public:
    /**
     *  Class constructor. Build all the time-invariant matrices used to compute the optimal output of the preview controller.
         @param period       Period (in ms) of the thread in which an object of this class is created.
         @param parameters   An object containing the main parameters. \see struct ZmpPreviewParams for details.
     */
    ZmpPreviewController(const int period, struct ZmpPreviewParams parameters);
    
    virtual ~ZmpPreviewController();
    
    bool initialize();
    
    /**
     *  Computes the optimal input, i.e.:
     *  \f[
         \mathbf{U}_{k+N_c|k} = (\mathbf{H}_p^T \mathbf{N}_b \mathbf{H}_p + \mathbf{N}_u + \mathbf{H}_h^T \mathbf{N}_w \mathbf{H}_h)^{-1} \left(\mathbf{H}^T_p \mathbf{N}_b (\mathbf{P}_r - \mathbf{G}_p \hat{\mathbf{h}}_k) + \mathbf{H}^T_h\mathbf{N}_w(\tilde{\mathbf{H}}_r - \mathbf{G}_h \hat{\mathbf{h}}_k)\right)
         \f]
     *  @param zmpRef    \f$\mathbf{P}_r\f$
     *  @param comVelRef \f$\tilde{\mathbf{H}}_r\f$
     *  @param hk        COM state at time \f$k\f$, i.e. \f$\hat{\mathbf{h}}_k\f$
     *  @param optimalU  Closed-form solution to the unconstrained QP problem, \f$\mathbf{U}_{k+N_c|k}\f$
     *
     *  @return True if computation is successful, false otherwise.
     */
    bool computeOptimalInput(Eigen::VectorXd zmpRef, Eigen::VectorXd comVelRef, Eigen::Vector2d hk, Eigen::VectorXd &optimalU);
    
    /**
     *  Builds \f$A_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Ah.
     */
    Eigen::MatrixXd buildAh(const double dt);
    
    /**
     *  Builds \f$B_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Bh.
     */
    Eigen::MatrixXd buildBh(const double dt);
    
    /**
     *  Builds \f$C_p\f$
     *
     *  @param cz Constant COM height.
     *  @param g  Constant gravity acceleration (m^2/s)
     *
     *  @return \f$C_p\f$
     */
    Eigen::MatrixXd buildCp(const double cz, const double g);
    
    /**
     *  Builds \f$G_p\f$
     *
     *  @param Cp \f$C_p\f$
     *  @param Ah \f$A_h\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$G_p\f$
     */
    Eigen::MatrixXd buildGp(Eigen::MatrixXd Cp, Eigen::MatrixXd Ah, const int Nc);
    
    /**
     *  Builds \f$H_p\f$
     *
     *  @param Cp \f$C_p\f$
     *  @param Bh \f$B_h\f$
     *  @param Ah \f$A_h\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$H_p\f$
     */
    Eigen::MatrixXd buildHp(Eigen::MatrixXd Cp, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc);
    
    /**
     *  Builds \f$C_h\f$
     *
     *  @return \f$C_h\f$
     */
    Eigen::MatrixXd buildCh();
    
    /**
     *  Builds \f$G_h\f$
     *
     *  @param Ch \f$C_h\f$
     *  @param Ah \f$A_h\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$G_h\f$
     */
    Eigen::MatrixXd buildGh(Eigen::MatrixXd Ch, Eigen::MatrixXd Ah, const int Nc);
    
    /**
     *  Builds \f$H_h\f$
     *
     *  @param Ch \f$C_h\f$
     *  @param Bh \f$B_h\f$
     *  @param Ah \f$A_h\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$H_h\f$
     */
    Eigen::MatrixXd buildHh(Eigen::MatrixXd Ch, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc);
    
    /**
     *  Builds \f$N_u\f$
     *
     *  @param nu \f$\eta_u\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$N_u\f$
     */
    Eigen::MatrixXd buildNu(const double nu, const int Nc);
    
    /**
     *  Builds \f$N_w\f$
     *
     *  @param nw \f$\eta_w\f$
     *  @param Nc \f$Nc\f$
     *
     *  @return \f$N_w\f$
     */
    Eigen::MatrixXd buildNw(const double nw, const int Nc);
    
    /**
     *  Builds \f$N_b\f$
     *
     *  @param nb \f$\eta_b\f$
     *  @param Nc \f$N_c\f$
     *
     *  @return \f$N_b\f$
     */
    Eigen::MatrixXd buildNb(const double nb, const int Nc);
    
private:
    /**
     *  CoM constant height. It is not hardcoded but corresponds to the vertical coordinate (height) of the CoM at the beginning of execution, i.e. \f$ c_z \f$.
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
     *  Size of preview window \f$ N_c \f$
     */
    const int Nc;
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
     *  Diagonal matrix \f$N_u\f$ of size \f$[2N_c \times 2N_c]\f$ weighting the input regularization term in the matricial expression of the controller's cost function.
     */
    const Eigen::MatrixXd Nu;
    /**
     *  Diagonal matrix \f$N_w\f$ of size \f$[2N_c \times 2N_c]\f$ weighting the walking cost function in its matrix form.
     */
    const Eigen::MatrixXd Nw;
    /**
     *  Diagonal matrix \f$N_b\f$ of size \f$[2N_c \times 2N_c]\f$ weighting the balancing cost function in its matrix form.
     */
    const Eigen::MatrixXd Nb;
    /**
     *  State matrix \f$\mathbf{A}_h\f$ from the linear state process of the COM state \f$\hat{\mathbf{h}}\f$. It is a constant matrix of size \f$6\times6\f$ equal to:
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
     *  Input matrix \f$\mathbf{B}_h\f$ from the linear state process of the CoM state \f$\hat{\mathbf{h}}\f$. It is constant of size \f$6\times2\f$ and equal to:
     \f[
     \mathbf{B_h} = \left[ \begin{array}{c}
     \delta^3/6 \\
     \delta t^2/2     \\
     \delta t
     \end{array} \right]
     \f]
     */
    const Eigen::MatrixXd Bh;
    /**
     *  Output matrix \f$C_p\f$ from the linear state process relating ZMP to the CoM dynamics \f$\hat{\mathbf{h}}\f$. It is time-invariant of size \f$2\times6\f$ and equal to:
     \f[
     \mathbf{B_h} = \left[ \begin{array}{c}
     \frac{\delta^3}{6}\mathbf{I}_2 \\
     \frac{\delta t^2}{2} \mathbf{I}_2     \\
     \delta t \mathbf{I}_2
     \end{array} \right]
     \f]
     */
    const Eigen::MatrixXd Cp;
    /**
     *  State matrix \f$\mathbf{G}_p\f$ from the preview horizon of ZMP outputs \f$\mathbf{P}\f$. It is of size \f$2Nc \times 6\f$ and equal to:
     \f[
     \mathbf{G}_p = \left[\begin{array}{c}
     \mathbf{C}_p\mathbf{A}_h \\
     \vdots \\
     \mathbf{C}_p\mathbf{A}^{N_c}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Gp;
    /**
     *  Input matrix \f$\mathbf{H}_p\f$ from the preview horizon of ZMP outputs \f$\mathbf{P}\f$. It is of size \f$2N_c \times 2N_c\f$ and equal to:
     \f[
     \mathbf{H}_p = \left[\begin{array}{cccc}
     \mathbf{C}_p\mathbf{B}_h               &   0                          &  \cdots   &   0 \\
     \mathbf{C}_p\mathbf{A}_h\mathbf{B}_h   &   \mathbf{C}_p\mathbf{B}_h   &  \cdots   &   0 \\
     \vdots                                 & \vdots                       & \ddots    &  \vdots \\
     \mathbf{C}_p\mathbf{A}^{N_c-1}_h\mathbf{B}_h & \mathbf{C}_p\mathbf{A}^{N_c-2}_h\mathbf{B}_h & \cdots & \mathbf{C}_p\mathbf{B}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Hp;
    /**
     *  Output matrix of the COM linear process state transition where the COM horizontal velocity is the output. It is of size \f$2\times6\f$ and equal to:
     \f[
     \mathbf{C}_h = \left[\begin{array}{ccc}
     \mathbf{0}_2  &  \mathbf{I}_2  &   \mathbf{0}_2 \\
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Ch;
    /**
     *  State matrix \f$\mathbf{G}_h\f$  of size \f$2N_c \times 6\f$ from the preview horizon of COM velocities. It is constant and equal to:
     \f[
     \mathbf{G}_p = \left[\begin{array}{c}
     \mathbf{C}_h\mathbf{A}_h \\
     \vdots \\
     \mathbf{C}_h\mathbf{A}^{N_c}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Gh;
    /**
     *  Input matrix \f$\mathbf{H}_h\f$ from the preview horizon of COM velocities \f$\tilde{\mathbf{H}}\f$, of size \f$2N_c \times 2N_c\f$ and equal to:
     \f[
     \mathbf{H}_p = \left[\begin{array}{cccc}
     \mathbf{C}_h\mathbf{B}_h               &   0                          &  \cdots   &   0 \\
     \mathbf{C}_h\mathbf{A}_h\mathbf{B}_h   &   \mathbf{C}_h\mathbf{B}_h   &  \cdots   &   0 \\
     \vdots                                 & \vdots                       & \ddots    &  \vdots \\
     \mathbf{C}_h\mathbf{A}^{N_c-1}_h\mathbf{B}_h & \mathbf{C}_h\mathbf{A}^{N_c-2}_h\mathbf{B}_h & \cdots & \mathbf{C}_h\mathbf{B}_h
     \end{array}\right]
     \f]
     */
    const Eigen::MatrixXd Hh;
    
    
    
    
};


struct ZmpPreviewParams {
    /**
     *  Size of preview window.
     */
    const double Nc;
    /**
     *  Robot's COM constant height.
     */
    const double cz;
    /**
     *  Weight of the input regularization term in the cost function \f$\eta_u\f$.
     */
    const double nu;
    /**
     *  Weight of the walking cost function \f$ \eta_w \f$.
     */
    const double nw;
    /**
     *  Weight of the balancing cost function \f$ \eta_b \f$.
     */
    const double nb;
    std::vector<Eigen::Vector2d> Pr;
    std::vector<Eigen::Vector2d> Hr;
    
    /**
     *  Helper function to transform Pr or Hr into a serialized vector of references.
     *
     *  @param ref vector of Nc references.
     *  @return Serialized references in one single column vector.
     *  @note Currently unused.
     */
    Eigen::MatrixXd serializeReference(std::vector<Eigen::Vector2d> ref) {
        Eigen::MatrixXd output;
        output.resize(2*Nc,1);
        unsigned int k = 0;
        for (auto it=ref.begin() ; it != ref.end(); ++it) {
            output.block(k*2, 0, 2, 1) = *it;
            k++;
        }
        return output;
    }
};

#endif
