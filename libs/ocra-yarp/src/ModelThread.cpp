/*! \file       ModelThread.cpp
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

#include <ocra-yarp/ModelThread.h>



using namespace ocra_yarp;

int ModelThread::MODEL_THREAD_COUNT = 0;

ModelThread::ModelThread(int period, const std::string& wbiConfFile, const std::string& givenRobotName, const bool isFloatingBase)
: RateThread(period)
, floatingBase(isFloatingBase)
, robotName(givenRobotName)
{
    uniqueModelThreadId = ++ModelThread::MODEL_THREAD_COUNT;
    yarp::os::Property yarpWbiOptions;
    yarpWbiOptions.fromConfigFile(wbiConfFile);
    // Overwrite the robot parameter that could be present in wbi_conf_file
    yarpWbiOptions.put("robot", robotName);
    // Initialize the WBI
    wbi = std::make_shared<yarpWbi::yarpWholeBodyInterface>(getModelThreadName().c_str(), yarpWbiOptions);
    // Add the robot's specific joints to the WBI.
    wbi::IDList robotJoints;
    std::string robotJointsListName = "ROBOT_MAIN_JOINTS";
    if(!yarpWbi::loadIdListFromConfig(robotJointsListName, yarpWbiOptions, robotJoints))
    {
        yLog.error() << "Impossible to load wbiId joint list with name: " << robotJointsListName;
    }
    wbi->addJoints(robotJoints);
}

ModelThread::ModelThread(int period, std::shared_ptr<wbi::wholeBodyInterface> wbiPtr, const std::string& givenRobotName, const bool isFloatingBase)
: RateThread(period)
, floatingBase(isFloatingBase)
, robotName(givenRobotName)
{
    uniqueModelThreadId = ++ModelThread::MODEL_THREAD_COUNT;
    wbi = wbiPtr;
}

ModelThread::~ModelThread()
{
}

std::string ModelThread::getModelThreadName()
{
    return "ModelThread_" + std::to_string(ModelThread::MODEL_THREAD_COUNT)+"/";
}

bool ModelThread::threadInit()
{
    if(wbi->init()){
        model = std::make_shared<OcraWbiModel>(robotName, wbi->getDoFs(), wbi, floatingBase);
        modelUpdater = std::make_shared<OcraWbiModelUpdater>(wbi, model);
        return true;
    } else {
        return false;
    }
}

void ModelThread::threadRelease()
{
    // delete model;
}

void ModelThread::run()
{
    modelUpdater->update();
}

const std::shared_ptr<ocra::Model> ModelThread::getModelPointer()
{
    return model;
}
