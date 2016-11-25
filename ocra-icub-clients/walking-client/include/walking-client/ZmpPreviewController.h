/**
 *  \class ZmpPreviewController
 *
 *  \brief Implementes an extended ZMP preview controller as an unconstrained QP problem.
 *
 *  \note Put original ZMP preview control reference along with Aurelien's
 *
 *  \author Jorhabib Eljaik
 *
 *  \details Given a rough ZMP trajectory and a corresponding consistent CoM velocity, this class computes at a fast rate optimal CoM jerks over a refined preview horizon. The ZMP preview control formulation in (reference goes here) is extended to account for a CoM tracking objective in order to minimize the error over a preview horizon to reference CoM velocities. This preview controller solves with

     \f[
        \mathcal{U}_{k+N|k} = (\mathbf{u}_{k|k}, ... , \mathbf{u})
     \f]

    Denoting a horizon of input CoM jerks \mathbf{u}, the following problem:

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

class ZmpPreviewController
{
public:
  ZmpPreviewController();
  ~ZmpPreviewController();

    bool initialize();
    bool initializeMatrices();
    bool buildCOMIntegrationMatrices();
    bool buildZMPPreviewMatrices();
    bool buildCOMVelPreviewMatrices();
    bool computeOptimalInput();

private:
};

#endif
