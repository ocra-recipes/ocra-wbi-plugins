/*! \file       ModelThread.h
 *  \brief      A class for launching generic control threads.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-yarp.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MODEL_THREAD_H
#define MODEL_THREAD_H

#include <string>
#include <yarp/os/Property.h>
#include <ocra-yarp/OcraWbiModel.h>
#include <ocra-yarp/OcraWbiModelUpdater.h>


namespace ocra_yarp
{
class ModelThread: public yarp::os::RateThread
{

public:
    // Constructor
    ModelThread(int period, const std::string& wbiConfFile, const bool isFloatingBase);
    ~ModelThread();


    // RateThread virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    //Public interface
    const ocra::Model* getModelPointer();


protected:

private:
    ocra::Model* model;
    std::shared_ptr<OcraWbiModelUpdater> modelUpdater;
    std::shared_ptr<wbi::wholeBodyInterface> wbi;

};
} // namespace ocra_yarp
#endif //MODEL_THREAD_H
