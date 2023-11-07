
#ifndef NAV_MATH_H
#define NAV_MATH_H

#include "Arduino.h"
#include "math.h"

#define RADIUS_OF_EARTH  6371e3  // in meters

typedef struct {
  float lon;
  float lat;
} GeographicPoint;

typedef struct {
  float across;
  float along;
} CrosstrackInfo;

double radiansToDegrees(double r);

double degreesToRadians(double r);

float mapF(float x, float in_min, float in_max, float out_min, float out_max);

float shortestSignedDistanceBetweenCircularValues(float origin, float target);

float calculateDistanceBetweenCoordinates(float lon1, float lat1, float lon2, float lat2);

float calculateInitialBearingBetweenCoordinates(float lon1, float lat1, float lon2, float lat2);

GeographicPoint calculateDestinationFromDistanceAndBearing(GeographicPoint start, float d, float bearing);
GeographicPoint calculateDestinationFromDistanceAndBearing2(float lon1, float lat1, float d, float bearing);

CrosstrackInfo calculateCrosstrackInfo(
   double lon1, double lat1, 
   double lon2, double lat2, 
   double lon3, double lat3
);

#endif
