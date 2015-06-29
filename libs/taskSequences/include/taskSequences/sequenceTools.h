#ifndef SEQUENCETOOLS_H
#define SEQUENCETOOLS_H

#include "wocra/Models/wOcraModel.h"
#include <Eigen/Dense>



#ifndef PI
#define PI 3.1415926
#endif


#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI/180.0
#endif

using namespace Eigen;

void getNominalPosture(wocra::wOcraModel &model, VectorXd &q);
void getHomePosture(wocra::wOcraModel &model, VectorXd &q);


#endif
