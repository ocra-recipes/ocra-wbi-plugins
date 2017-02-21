/*! \file       Constraint.h
 *  \brief      A constraint base class.
 *  \details
 *  \author     [Jorhabib Eljaik](https://github.com/jeljaik)
 *  \date       Feb 2017
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-recipes.
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


#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include <walking-client/utils.h>
#include <Eigen/Core>
#include "unsupported/Eigen/MatrixFunctions"
#include <ocra/util/ErrorsHelper.h>
#include <memory>

#define SIZE_STATE_VECTOR 16
#define SIZE_INPUT_VECTOR 12

class Constraint {
public:
    Constraint() { }
    void init() { OCRA_WARNING("Trying to initialize"); buildMatrixCi(); buildMatrixCii(); buildVectord(); };
    Eigen::MatrixXd getCi() { return _Ci; };
    Eigen::MatrixXd getCii() { return _Cii; };
    Eigen::VectorXd getd(){ return _d; };
protected:
    Eigen::MatrixXd _Ci;
    Eigen::MatrixXd _Cii;
    Eigen::VectorXd _d;
protected:
    virtual void buildMatrixCi(){ OCRA_WARNING("Method not implemented yet") };
    virtual void buildMatrixCii(){ OCRA_WARNING("Method not implemented yet") };
    virtual void buildVectord(){ OCRA_WARNING("Method not implemented yet") };
};

#endif
