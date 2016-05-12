#ifndef SEQUENCELIBRARY_H
#define SEQUENCELIBRARY_H




#include "sequences/FixedBaseMinimalTasks.h"
#include "sequences/FloatingBaseMinimalTasks.h"
#include "sequences/FloatingBaseCoMBalancing.h"
#include "sequences/InitialPoseHold.h"
#include "sequences/NominalPose.h"
#include "sequences/LeftHandReach.h"
#include "sequences/LeftRightHandReach.h"
#include "sequences/CartesianTest.h"
#include "sequences/PoseTest.h"
#include "sequences/OrientationTest.h"
#include "sequences/TrajectoryTrackingTest.h"
#include "sequences/FloatingBaseEstimationTests.h"
#include "sequences/JointTest.h"
#include "sequences/Debug.h"
#include "sequences/Empty.h"
#include "sequences/Exploration.h"

#if USING_SMLT
#include "sequences/TaskOptimization.h"
#include "sequences/StandingReach.h"
#include "sequences/MoveWeight.h"
#include "sequences/Experiment.h"
#endif
// namespace sequence{

    ocra::TaskSequence* LoadSequence(const std::string& name);

// }


#endif // SEQUENCELIBRARY_H
