#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "taskSequences/sequenceLibrary.h"


// namespace sequence{

    wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name)
    {
        function_list["FixedBaseMinimalTasks"]        = new FixedBaseMinimalTasks();
        function_list["FloatingBaseMinimalTasks"]     = new FloatingBaseMinimalTasks();
        function_list["InitialPoseHold"]              = new InitialPoseHold();
        function_list["NominalPose"]                  = new NominalPose();
        function_list["LeftHandReach"]                = new LeftHandReach();
        function_list["LeftRightHandReach"]           = new LeftRightHandReach();
        function_list["CartesianTest"]                = new CartesianTest();
        function_list["PoseTest"]                     = new PoseTest();
        function_list["OrientationTest"]              = new OrientationTest();
        function_list["TrajectoryTrackingTest"]       = new TrajectoryTrackingTest();
        function_list["FloatingBaseEstimationTests"]  = new FloatingBaseEstimationTests();
        function_list["JointTest"]                    = new JointTest();
        function_list["Debug"]                        = new Debug();
        function_list["Empty"]                        = new Empty();

        if(function_list.find(name) != function_list.end())
        {
            std::cout << "[LoadSequence()]: Sequence " << name << " was successfully loaded." << std::endl;
            return function_list[name];
        } else
        {
            map_sequence::iterator it;
            std::string errorMessage = "[LoadSequence()]: Error! Sequence '" + name + "' cannot be found. The following names are valid:";
            for (it = function_list.begin(); it != function_list.end(); ++it)
            {
              errorMessage+=("\n - " + it->first);
            }
            throw std::runtime_error(errorMessage);
            return NULL;
        }

        // TODO: using the map class the list is clearer but we still need to list all the existing functions
        // what we need is something like the "eval" function existing on MATLAB and similar softwares. It exists?
    }
// }
