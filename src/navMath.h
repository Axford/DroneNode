
#ifndef NAV_MATH_H
#define NAV_MATH_H

#include "Arduino.h"
#include "math.h"

#define RADIUS_OF_EARTH  6371e3  // in meters

typedef struct {
  float lon;
  float lat;
} GeographicPoint;

double radiansToDegrees(double r);

double degreesToRadians(double r);

float mapF(float x, float in_min, float in_max, float out_min, float out_max);

float shortestSignedDistanceBetweenCircularValues(float origin, float target);

float calculateDistanceBetweenCoordinates(float lon1, float lat1, float lon2, float lat2);

float calculateInitialBearingBetweenCoordinates(float lon1, float lat1, float lon2, float lat2);

GeographicPoint calculateDestinationFromDistanceAndBearing(GeographicPoint start, float d, float bearing);

#endif
