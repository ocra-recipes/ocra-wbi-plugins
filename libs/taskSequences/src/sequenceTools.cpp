#include "taskSequences/sequenceTools.h"

void getNominalPosture(wocra::wOcraModel& model, VectorXd &q)
{
  q[model.getDofIndex("torso_pitch")] = M_PI / 18;
  q[model.getDofIndex("r_elbow")] = M_PI / 4;
  q[model.getDofIndex("l_elbow")] = M_PI / 4;
  q[model.getDofIndex("l_shoulder_roll")] = M_PI / 6;
  q[model.getDofIndex("r_shoulder_roll")] = M_PI / 6;
  q[model.getDofIndex("l_shoulder_pitch")] = -M_PI / 6;
  q[model.getDofIndex("r_shoulder_pitch")] = -M_PI / 6;
  q[model.getDofIndex("l_hip_pitch")] = M_PI / 8;
  q[model.getDofIndex("r_hip_pitch")] = M_PI / 8;
  q[model.getDofIndex("l_hip_roll")] = M_PI / 18;
  q[model.getDofIndex("r_hip_roll")] = M_PI / 18;
  q[model.getDofIndex("l_knee")] = -M_PI / 6;
  q[model.getDofIndex("r_knee")] = -M_PI / 6;
}

void getHomePosture(wocra::wOcraModel& model, VectorXd &q)
{
  q[model.getDofIndex("l_shoulder_pitch")] = -80*DEG_TO_RAD;
  q[model.getDofIndex("l_shoulder_roll")] = 20*DEG_TO_RAD;
  q[model.getDofIndex("l_shoulder_yaw")]   =  0.0*DEG_TO_RAD;
  q[model.getDofIndex("l_elbow")]          =  30.0*DEG_TO_RAD;//PI/4.0;

  q[model.getDofIndex("r_shoulder_pitch")] = -80*DEG_TO_RAD;
  q[model.getDofIndex("r_shoulder_roll")] = 20*DEG_TO_RAD;
  q[model.getDofIndex("r_shoulder_yaw")]   =  0.0*DEG_TO_RAD;
  q[model.getDofIndex("r_elbow")]          =  30.0*DEG_TO_RAD;//PI/4.0;
}

void raiseArm(char s, wocra::wOcraModel &model, VectorXd &q)
{
  std::string side(&s, 1);
  q[model.getDofIndex(side + "_shoulder_pitch")] = -80.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_shoulder_roll")]  =  20.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_shoulder_yaw")]   =  0.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_elbow")]          =  30.0*DEG_TO_RAD;
}

void downArm(char s, wocra::wOcraModel &model, VectorXd &q)
{
  std::string side(&s, 1);
  q[model.getDofIndex(side + "_shoulder_roll")] = 0.0;
  q[model.getDofIndex(side + "_shoulder_pitch")] = -M_PI / 6;
  q[model.getDofIndex(side + "_shoulder_yaw")]   =  0.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_elbow")] = M_PI / 4;
}

void extendArm(char s, wocra::wOcraModel &model, VectorXd &q)
{
  std::string side(&s, 1);
  q[model.getDofIndex(side + "_shoulder_pitch")] = -89.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_shoulder_roll")]  =  20.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_shoulder_yaw")]   =  90.0*DEG_TO_RAD;
  q[model.getDofIndex(side + "_elbow")]          =  2.0*DEG_TO_RAD;
}
