#ifndef OBSERVERBASE_H
#define OBSERVERBASE_H

#include <iostream>
#include <fstream>


class observerBase
{
public:

    observerBase();
    virtual ~observerBase();

    // void sample(const double time, wocra::wOcraModel& state);

protected:

    std::fstream dumpFile;





};


#endif
