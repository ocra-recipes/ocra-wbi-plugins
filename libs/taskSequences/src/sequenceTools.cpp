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
    // q[model.getDofIndex("l_hip_pitch")] = M_PI / 8;
    // q[model.getDofIndex("r_hip_pitch")] = M_PI / 8;
    // q[model.getDofIndex("l_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("r_hip_roll")] = M_PI / 18;
    // q[model.getDofIndex("l_knee")] = -M_PI / 6;
    q[model.getDofIndex("r_knee")] = -M_PI / 6;
}

void getHomePosture(wocra::wOcraModel& model, VectorXd &q)
{
    q[model.getDofIndex("l_shoulder_roll")]=   20.0*DEG_TO_RAD;//PI/8.0;
    q[model.getDofIndex("r_shoulder_roll")]=   20.0*DEG_TO_RAD;//PI/8.0;
    q[model.getDofIndex("l_shoulder_pitch")]=  -25.0*DEG_TO_RAD;//-PI/8.0;
    q[model.getDofIndex("r_shoulder_pitch")]=  -25.0*DEG_TO_RAD;//-PI/8.0;
    q[model.getDofIndex("l_elbow")]        =   50.0*DEG_TO_RAD;//PI/4.0;
    q[model.getDofIndex("r_elbow")]        =   50.0*DEG_TO_RAD;//PI/4.0;
}
