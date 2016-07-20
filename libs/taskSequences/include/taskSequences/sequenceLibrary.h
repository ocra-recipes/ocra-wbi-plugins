#ifndef SEQUENCELIBRARY_H
#define SEQUENCELIBRARY_H




#include "sequences/FixedBaseMinimalTasks.h"
#include "sequences/FloatingBaseMinimalTasks.h"
#include "sequences/Debug.h"
#include "sequences/Empty.h"

#if USING_SMLT
#include "sequences/ObstacleAvoidance.h"
#include "sequences/MoveWeight.h"
#endif
// namespace sequence{

    wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name);

// }


#endif // SEQUENCELIBRARY_H
