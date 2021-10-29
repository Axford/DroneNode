
#ifndef NAV_MATH_H
#define NAV_MATH_H

#include "Arduino.h"
#include "math.h"

#define RADIUS_OF_EARTH  6371e3  // in meters

double radiansToDegrees(double r);

double degreesToRadians(double r);

float shortestSignedDistanceBetweenCircularValues(float origin, float target);


#endif
