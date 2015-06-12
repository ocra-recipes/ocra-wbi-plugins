#include "ISIRWholeBodyController/sequenceCollection.h"

#include "ISIRWholeBodyController/sequenceLibrary.h"

wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name)
{
    if (name == "Sequence_InitialPoseHold")
        return new Sequence_InitialPoseHold();
    else if (name == "Sequence_NominalPose")
        return new Sequence_NominalPose();
    else if (name == "Sequence_LeftHandReach")
        return new Sequence_LeftHandReach();
    else if (name == "Sequence_LeftRightHandReach")
        return new Sequence_LeftRightHandReach();
    else if (name == "Sequence_CartesianTest")
        return new Sequence_CartesianTest();
    else if (name == "Sequence_PoseTest")
        return new Sequence_PoseTest();
    else if (name == "Sequence_OrientationTest")
        return new Sequence_OrientationTest();
    else if (name == "Sequence_TrajectoryTrackingTest")
        return new Sequence_TrajectoryTrackingTest();
    else if (name == "Sequence_FloatingBaseEstimationTests")
        return new Sequence_FloatingBaseEstimationTests();
    else if (name == "Sequence_JointTest")
        return new Sequence_JointTest();
    else
        throw std::runtime_error(std::string("[LoadSequence()]: Error - Sequence name cannot be found."));
}
