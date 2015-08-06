#ifndef SEQUENCELIBRARY_H
#define SEQUENCELIBRARY_H

#include <map>

#include "sequences/FixedBaseMinimalTasks.h"
#include "sequences/FloatingBaseMinimalTasks.h"
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



// namespace sequence{
    typedef std::map<std::string, wocra::wOcraTaskSequenceBase*> map_sequence;
    map_sequence function_list;
    wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name);
// }


#endif // SEQUENCELIBRARY_H
