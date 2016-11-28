/**
 *  \class ZmpController
 *
 *  \brief Implementes a ZMP controller as a force set point regulator.
 *
 *  \note Put original ZMP preview control reference along with Aurelien's
 *
 *  \author Jorhabib Eljaik
 *
 *  \cite krause2012stabilization
 *
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *
 *  \details In \cite{krause2012stabilization} a position-based ZMP controller was implemented by  relating the desired ZMP \f$ \mathbf{p}_d \f$ to a desired force, given the relationship between CoM acceleration and the ZMP. When the dynamical effects of the vertical motion of the robot together with the change of angular momentum are neglected, the horizontal location of the ZMP \f$\mathbf{p} = (p_x, p_y)\f$ can be computed as:
 \f{equation}
 \ddot{h} = \omega^2(\mathbf{h} - \mathbf{p})
 \label{eq:simplifiedZMP}
 \f}
 Where \f$\omega = \sqrt{g/c_z}\f$ as done in \cite kajita2003biped. Therefore the desired \f$\mathbf{p}_d\f$ can be associated to a desire force on the CoM:
 \f[
 F_d = m \omega^2 (\mathbf{h} - \mathbf{p}_d)
 \f]
 Where \f$m\f$ is the total mass of the robot and a force set point regulator can be implemented as:
 \f[
 \dot{\mathbf{h}}_d = k_f(F_d - F)
 \f]
 Where \f$k_f > 0\f$ is a force control gain. Substituting \ref{eq:simplifiedZMP} we get:
 \f[
 \dot{\mathbf{h}}_d = k_f m \omega^2(\mathbf{p} - \mathbf{p}_d)
 \f]
 */

#ifndef _ZMPCONTROLLER_
#define _ZMPCONTROLLER_

#include <ocra-icub/Utilities.h>
#include <ocra/util/ErrorsHelper.h>
#include <Eigen/Dense>
#include <vector>


class ZmpController
{
public:

    ZmpController(const int period, struct ZmpControllerParams parameters);

    virtual ~ZmpController();
    
    /**
     *  Compute the instantaneous desired horizonal COM velocity for the corresponding desired zmp position.
     *
     *  @param pd Horizontal desired zmp position
     *  @param[out] dhd Horizonal CoM velocity \f$\dot{\mathbf{h}}_d\f$
     */
    bool computehd(Eigen::Vector2d pd, Eigen::Vector2d &dhd);

};

struct ZmpControllerParams {
    const double kf;
    const double m;
    const double cz;
    const double g;
};

#endif
