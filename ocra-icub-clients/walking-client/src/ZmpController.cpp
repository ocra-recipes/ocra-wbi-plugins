#include "walking-client/ZmpController.h"

ZmpController::ZmpController(const int period, ZmpControllerParams parameters) :
params(parameters)
{ }

ZmpController::~ZmpController() {
}

bool ZmpController::computehd(Eigen::Vector2d p, Eigen::Vector2d pd, Eigen::Vector2d &dhd){
    dhd = params.kf * params.m * (params.cz/params.g) * ( p - pd );
    return true;
}

