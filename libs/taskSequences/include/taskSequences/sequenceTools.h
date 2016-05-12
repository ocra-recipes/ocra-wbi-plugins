#ifndef SEQUENCETOOLS_H
#define SEQUENCETOOLS_H

#include "ocra/control/Model.h"
#include <Eigen/Dense>



#ifndef PI
#define PI 3.1415926
#endif


#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI/180.0
#endif

using namespace Eigen;

void getNominalPosture(ocra::Model &model, VectorXd &q);
void getHomePosture(ocra::Model &model, VectorXd &q);


#endif
