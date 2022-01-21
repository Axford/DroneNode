#include "navMath.h"

/*
Nav math from: https://www.movable-type.co.uk/scripts/latlong.html
*/

double radiansToDegrees(double r) {
  return r * 180 / PI;
}

double degreesToRadians(double r) {
  return r * PI /  180;
}

float shortestSignedDistanceBetweenCircularValues(float origin, float target){
  float signedDiff = 0.0;
  float raw_diff = origin > target ? origin - target : target - origin;
  float mod_diff = fmod(raw_diff, 360); //equates rollover values. E.g 0 == 360 degrees in circle

  if(mod_diff > (360/2) ){
    //There is a shorter path in opposite direction
    signedDiff = (360 - mod_diff);
    if(target>origin) signedDiff = signedDiff * -1;
  } else {
    signedDiff = mod_diff;
    if(origin>target) signedDiff = signedDiff * -1;
  }

  return signedDiff;
}


float calculateDistanceBetweenCoordinates(float lon1, float lat1, float lon2, float lat2) {
  float R = RADIUS_OF_EARTH; // metres
  float lat1r = lat1 * PI/180; // φ, λ in radians
  float lat2r = lat2 * PI/180;
  float lon1r = lon1 * PI/180; // φ, λ in radians
  float lon2r = lon2 * PI/180;
  //float dlat = (lat2-lat1) * PI/180;
  //float dlon = (lon2-lon1) * PI/180;

  /*
  const x = (λ2-λ1) * Math.cos((φ1+φ2)/2);
  const y = (φ2-φ1);
  const d = Math.sqrt(x*x + y*y) * R;
  */
  float x = (lon2r-lon1r) * cos((lat1r+lat2r)/2);
  float y = (lat2r-lat1r);
  float d = sqrt(x*x + y*y) * R;

  return d;
}


float calculateInitialBearingBetweenCoordinates(float lon1, float lat1, float lon2, float lat2) {
  float lat1r = lat1 * PI/180; // φ, λ in radians
  float lat2r = lat2 * PI/180;
  float lon1r = lon1 * PI/180; // φ, λ in radians
  float lon2r = lon2 * PI/180;

  float y = sin(lon2r-lon1r) * cos(lat2r);
  float x = cos(lat1r)*sin(lat2r) - sin(lat1r)*cos(lat2r)*cos(lon2r-lon1r);
  float ang = atan2(y, x);
  float bearing = fmod((ang * 180 / PI + 360), 360);
  return bearing;
}

//Destination point given distance and bearing from start point
/*
const φ2 = Math.asin( Math.sin(φ1)*Math.cos(d/R) +
                      Math.cos(φ1)*Math.sin(d/R)*Math.cos(brng) );
const λ2 = λ1 + Math.atan2(Math.sin(brng)*Math.sin(d/R)*Math.cos(φ1),
                           Math.cos(d/R)-Math.sin(φ1)*Math.sin(φ2));
*/
GeographicPoint calculateDestinationFromDistanceAndBearing(GeographicPoint start, float d, float bearing) {
  GeographicPoint p;
  float R = RADIUS_OF_EARTH; // metres
  float lat1r = start.lat * PI/180; // φ, λ in radians
  float lon1r = start.lon * PI/180;
  float br = bearing * PI/180;

  float a = sin(lat1r)*cos(d/R) + cos(lat1r)*sin(d/R)*cos(br);
  p.lat = asin( a );
  p.lon = lon1r + atan2(
    sin(br)*sin(d/R)*cos(lat1r),
    cos(d/R) - sin(lat1r)*a
  );
  // convert to degrees
  p.lat = p.lat * 180/PI;
  p.lon = p.lon * 180/PI;
  // normalise lon
  p.lon = fmod((p.lon + 540),360) - 180;
  return p;
}
