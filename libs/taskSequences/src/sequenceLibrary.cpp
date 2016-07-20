#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "taskSequences/sequenceLibrary.h"

#if USING_SMLT
#define SMLT_SEQUENCES else if (name == "ObstacleAvoidance"){return new ObstacleAvoidance();} else if (name == "MoveWeight"){return new MoveWeight();}
#else
#define SMLT_SEQUENCES
#endif

#if USING_SMLT
#define SMLT_SEQUENCE_NAME_ERROR errorMessage+="\nObstacleAvoidance"; errorMessage+="\nMoveWeight";
#else
#define SMLT_SEQUENCE_NAME_ERROR
#endif

// namespace sequence{

    wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name)
    {
        //Base Sequences
        if (name == "FixedBaseMinimalTasks")
            return new FixedBaseMinimalTasks();
        else if (name == "FloatingBaseMinimalTasks")
            return new FloatingBaseMinimalTasks();

        //Cpp Sequences
        else if (name == "Debug"){return new Debug();}
        else if (name == "Empty"){return new Empty();}

        SMLT_SEQUENCES

        // TODO: It would be nice to handle errors a little more gently here and rather than throwing an error just not create any sequence. This could be done with a separate function doing a string check in thread.cpp. This however adds code. What would be nice is to just have a vector of the different sequence names and be able to figure out the constructor progrmatically rather than having to write it explicitly. Not sure if this is possible.

        else{
            std::string errorMessage = "[LoadSequence()]: Error - Sequence name cannot be found. The following names are valid:";

            errorMessage+="\nFixedBaseMinimalTasks";
            errorMessage+="\nFloatingBaseMinimalTasks";
            errorMessage+="\nDebug";
            errorMessage+="\nEmpty";
            SMLT_SEQUENCE_NAME_ERROR
            throw std::runtime_error(errorMessage);
        }
    }
// }
