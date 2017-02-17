/**
 *  \class ZmpController
 *
 *  \brief Implementes a ZMP controller as a force set point regulator.
 *
 *  \author Jorhabib Eljaik
 *
 *  \cite krause2012stabilization
 *
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *
 *  \details In \cite krause2012stabilization a position-based ZMP controller was implemented by  relating the desired ZMP \f$ \mathbf{p}_d \f$ to a desired force, given the relationship between CoM acceleration and the ZMP. When the dynamical effects of the vertical motion of the robot together with the change of angular momentum are neglected, the horizontal location of the ZMP \f$\mathbf{p} = (p_x, p_y)\f$ can be computed as:
 \anchor simplifiedZMP
 \f{equation}
 \ddot{h} = \omega^2(\mathbf{h} - \mathbf{p})
 \f}

 Where \f$\omega = \sqrt{g/c_z}\f$ as done in \cite kajita2003biped. Therefore the desired \f$\mathbf{p}_d\f$ can be associated to a desired force on the CoM:
 \f[
 F_d = m \omega^2 (\mathbf{h} - \mathbf{p}_d)
 \f]
 Where \f$m\f$ is the total mass of the robot and a force set point regulator can be implemented as:
 \f[
 \dot{\mathbf{h}}_d = k_f(F_d - F)
 \f]
 Where \f$k_f > 0\f$ is a force control gain. Substituting \ref simplifiedZMP we get:
 \f[
 \dot{\mathbf{h}}_d = k_f m \omega^2(\mathbf{p} - \mathbf{p}_d)
 \f]
 
 @sa computehd()
 */

#ifndef _ZMPCONTROLLER_
#define _ZMPCONTROLLER_

#include <ocra-icub/Utilities.h>
#include <ocra/util/ErrorsHelper.h>
#include <ocra/util/EigenUtilities.h>
#include <ocra/control/TaskState.h>
#include <ocra-recipes/TaskConnection.h>
#include <Eigen/Dense>
#include <vector>

struct ZmpControllerParams {
    /**
     *  Positive force control gain in the x direction
     */
    double kfx;
    /**
     *  Positive force control gain in the y direction
     */
    double kfy;
    double kdx;
    double kdy;
    /**
     *  Total mass of the robot
     */
    double m;
    /**
     *  CoM height
     */
    double cz;
    /**
     *  Gravity acceleration = 9.8m/s^2
     */
    double g;
    /**
     *  Controller period in seconds
     */
    double controllerPeriod;
    
    ZmpControllerParams(double kfx,
                        double kfy,
                        double kdx,
                        double kdy,
                        double m,
                        double cz,
                        double g,
                        double controllerPeriod):
    kfx(kfx),
    kfy(kfy),
    kdx(kdx),
    kdy(kdy),
    m(m),
    cz(cz),
    g(g),
    controllerPeriod(controllerPeriod){}
};

enum FOOT {
    LEFT_FOOT,
    RIGHT_FOOT
};

class ZmpController
{
public:

    /**
     * @todo Deprecate this class
     */
    ZmpController(const int period,
                  std::shared_ptr<ocra::Model> modelPtr,
                  std::shared_ptr<ZmpControllerParams> parameters);

    virtual ~ZmpController();
    
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
                                     Eigen::Vector2d &globalZMP);
    
    /**
     *  Retrieves the FT sensor adjoint matrix expressed in the world reference frame which multiplied by the local measurement of the sensor gives you the measurement in the world reference.
     *
     *  @param whichFoot LEFT_FOOT or RIGHT_FOOT
     *  @param[out] T         Adjoint matrix.
     */
    void getFTSensorAdjointMatrix(FOOT whichFoot, Eigen::MatrixXd &T, Eigen::Vector3d &sensorPosition);
    
    void getLeftFootPosition(Eigen::Vector3d &leftFootPosition);
    
    void getRightFootPosition(Eigen::Vector3d &rightFootPosition);
    

    
    /**
     *  Computes the instantaneous desired horizontal COM velocity for the corresponding desired zmp position.
     *
     *  @param p  Horizonal measured zmp position
     *  @param pd Horizontal desired zmp position
     *  @param[out] dhd Horizonal CoM velocity \f$\dot{\mathbf{h}}_d\f$
     */
    bool computehd(Eigen::Vector2d p, Eigen::Vector2d pd, Eigen::Vector2d &dhd);
    
    void computehdd(Eigen::Vector3d comPosition, Eigen::Vector2d globalZMP, Eigen::Vector2d &ddh);
    
    void computeh(Eigen::Vector2d prevComPosition, Eigen::Vector2d prevComVel, Eigen::Vector2d &intComPosition);
    
    ocra::TaskState createDesiredState(Eigen::Vector2d comRefPosition, Eigen::Vector2d comRefVelocity, Eigen::Vector2d comRefAcceleration);
private:
    std::shared_ptr<ZmpControllerParams> _params;
    std::shared_ptr<ocra::Model> _model;

};


#endif
