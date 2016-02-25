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


ModelThread::ModelThread(int period, const std::string& wbiConfFile, const bool isFloatingBase):
RateThread(period),
model(NULL)
{
    yarp::os::Property yarpWbiOptions;
    yarpWbiOptions.fromConfigFile(wbiConfFile);
    wbi = std::make_shared<yarpWbi::yarpWholeBodyInterface>("modelthread", yarpWbiOptions);
    model = new OcraWbiModel(yarpWbiOptions.find("robot").asString(), wbi->getDoFs(), wbi, isFloatingBase);
    modelUpdater = std::make_shared<OcraWbiModelUpdater>();
}

ModelThread::~ModelThread()
{
}


bool ModelThread::threadInit()
{
    modelUpdater->initialize(wbi, model);
}

void ModelThread::threadRelease()
{
    delete model;
}

void ModelThread::run()
{
    modelUpdater->update(wbi, model);
}

const ocra::Model* ModelThread::getModelPointer()
{
    return model;
}
