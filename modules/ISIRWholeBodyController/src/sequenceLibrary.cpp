#include "ISIRWholeBodyController/sequenceCollection.h"

#include "ISIRWholeBodyController/sequenceLibrary.h"

wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name)
{
    //Base Sequences
    if (name == "Sequence_FixedBaseMinimalTasks")
        return new Sequence_FixedBaseMinimalTasks();
    else if (name == "Sequence_FloatingBaseMinimalTasks")
        return new Sequence_FloatingBaseMinimalTasks();

    //Cpp Sequences
    else if (name == "Sequence_InitialPoseHold")
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

    // TODO: It would be nice to handle errors a little more gently here and rather than throwing an error just not create any sequence. This could be done with a separate function doing a string check in thread.cpp. This however adds code. What would be nice is to just have a vector of the different sequence names and be able to figure out the constructor progrmatically rather than having to write it explicitly. Not sure if this is possible.

    else{
        std::string errorMessage = "[LoadSequence()]: Error - Sequence name cannot be found. The following names are valid:";

        errorMessage+="\nSequence_FixedBaseMinimalTasks";
        errorMessage+="\nSequence_FloatingBaseMinimalTasks";
        errorMessage+="\nSequence_InitialPoseHold";
        errorMessage+="\nSequence_NominalPose";
        errorMessage+="\nSequence_LeftHandReach";
        errorMessage+="\nSequence_LeftRightHandReach";
        errorMessage+="\nSequence_CartesianTest";
        errorMessage+="\nSequence_PoseTest";
        errorMessage+="\nSequence_OrientationTest";
        errorMessage+="\nSequence_TrajectoryTrackingTest";
        errorMessage+="\nSequence_FloatingBaseEstimationTests";
        errorMessage+="\nSequence_JointTest";
        throw std::runtime_error(errorMessage);
    }
}
