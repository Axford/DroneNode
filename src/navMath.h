
#ifndef NAV_MATH_H
#define NAV_MATH_H

#include "Arduino.h"
#include "math.h"

#define RADIUS_OF_EARTH  6371e3  // in meters

double radiansToDegrees(double r);

double degreesToRadians(double r);

float shortestSignedDistanceBetweenCircularValues(float origin, float target);

float calculateDistanceBetweenCoordinates(float lon1, float lat1, float lon2, float lat2);

#endif
