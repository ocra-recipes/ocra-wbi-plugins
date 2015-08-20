#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "taskSequences/sequenceLibrary.h"


// namespace sequence{

    wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name)
    {
        //Base Sequences
        if (name == "FixedBaseMinimalTasks")
            return new FixedBaseMinimalTasks();
        else if (name == "FloatingBaseMinimalTasks")
            return new FloatingBaseMinimalTasks();

        //Cpp Sequences
        else if (name == "FloatingBaseCoMBalancing")
            return new FloatingBaseCoMBalancing();
        else if (name == "InitialPoseHold")
            return new InitialPoseHold();
        else if (name == "NominalPose")
            return new NominalPose();
        else if (name == "LeftHandReach")
            return new LeftHandReach();
        else if (name == "LeftRightHandReach")
            return new LeftRightHandReach();
        else if (name == "CartesianTest")
            return new CartesianTest();
        else if (name == "PoseTest")
            return new PoseTest();
        else if (name == "OrientationTest")
            return new OrientationTest();
        else if (name == "TrajectoryTrackingTest")
            return new TrajectoryTrackingTest();
        else if (name == "FloatingBaseEstimationTests")
            return new FloatingBaseEstimationTests();
        else if (name == "JointTest")
            return new JointTest();
        else if (name == "Debug")
            return new Debug();
        else if (name == "Empty")
            return new Empty();

        // TODO: It would be nice to handle errors a little more gently here and rather than throwing an error just not create any sequence. This could be done with a separate function doing a string check in thread.cpp. This however adds code. What would be nice is to just have a vector of the different sequence names and be able to figure out the constructor progrmatically rather than having to write it explicitly. Not sure if this is possible.

        else{
            std::string errorMessage = "[LoadSequence()]: Error - Sequence name cannot be found. The following names are valid:";

            errorMessage+="\nFixedBaseMinimalTasks";
            errorMessage+="\nFloatingBaseMinimalTasks";
            errorMessage+="\nFloatingBaseCoMBalancing";
            errorMessage+="\nInitialPoseHold";
            errorMessage+="\nNominalPose";
            errorMessage+="\nLeftHandReach";
            errorMessage+="\nLeftRightHandReach";
            errorMessage+="\nCartesianTest";
            errorMessage+="\nPoseTest";
            errorMessage+="\nOrientationTest";
            errorMessage+="\nTrajectoryTrackingTest";
            errorMessage+="\nFloatingBaseEstimationTests";
            errorMessage+="\nJointTest";
            errorMessage+="\nDebug";
            errorMessage+="\nEmpty";
            throw std::runtime_error(errorMessage);
        }
    }
// }
