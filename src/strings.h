#ifndef STRINGS
#define STRINGS

#include <Arduino.h>

// strings
static const char STRING_D[]  = " %d ";
static const char STRING_F[]  = " %F ";
static const char STRING_S[]  = " %s ";
static const char STRING_X[]  = " %X ";

static const char STRING_BLANK[]  = "";


static const char STRING_ACCEL[] = "accel";
static const char STRING_ACTIVESCENE[] = "activeScene";
static const char STRING_ADDR[] = "addr";
static const char STRING_ADJ_HEADING[] = "adjHeading";
static const char STRING_ADJ_TARGET[] = "adjTarget";
static const char STRING_ALARM[] = "alarm";
static const char STRING_ALTITUDE[] = "altitude";
static const char STRING_ANALOG[] = "analog";
static const char STRING_AOA[] = "AOA";
static const char STRING_ARM[] = "arm";
static const char STRING_BAUD[] = "baud";
static const char STRING_BRIGHTNESS[] = "brightness";
static const char STRING_BROADCAST[] = "broadcast";
static const char STRING_BUILD[] = "build";
static const char STRING_BUS[] = "bus";
static const char STRING_BUSV[] = "busV";
static const char STRING_BUTTON[] = "button";
static const char STRING_CALIB_X[] = "calibX";
static const char STRING_CALIB_Y[] = "calibY";
static const char STRING_CALIB_Z[] = "calibZ";
static const char STRING_CALIB_G[] = "calibG";
static const char STRING_CANCEL[] = "cancel";
static const char STRING_CELLS[] = "cells";
static const char STRING_CELLV[] = "cellV";
static const char STRING_CENTRE[] = "centre";
static const char STRING_CHANNELS[] = "channels";
static const char STRING_CHOKED[] = "choked";
static const char STRING_CORRECTION[] = "correction";
static const char STRING_COURSE[] = "course";
static const char STRING_COW[] = "COW"; // course over water
static const char STRING_CPU[] = "CPU";
static const char STRING_CROSSTRACK[] = "crosstrack";
static const char STRING_CROSSWIND[] = "crosswind";
static const char STRING_CURRENT[] = "current";
static const char STRING_DEADBAND[] = "deadband";
static const char STRING_DECLINATION[] = "declination";
static const char STRING_DEBUG[] = "debug";
static const char STRING_DELAY[] = "delay";
static const char STRING_DEPTH[] = "depth";
static const char STRING_DIRECTION[] = "direction";
static const char STRING_DISTANCE[] = "distance";
static const char STRING_DISCOVERY[] = "discovery";
static const char STRING_DRONE[] = "Drone";
static const char STRING_ERROR[] = "error";
static const char STRING_ENABLE[] = "enable";
static const char STRING_ETC[] = "ETC";
static const char STRING_FALSE[] = "false";
static const char STRING_FIX[] = "fix";
static const char STRING_FLAGS[] = "flags";
static const char STRING_FOLLOWME[] = "followMe";
static const char STRING_FREQUENCY[] = "frequency";
static const char STRING_GYRO[] = "gyro";
static const char STRING_HDOP[] = "HDOP";
static const char STRING_HEADING[] = "heading";
static const char STRING_HEAP[] = "heap";
static const char STRING_HOME[] = "home";
static const char STRING_HOSTNAME[] = "hostname";
static const char STRING_HUMIDITY[] = "humidity";
static const char STRING_INPUT[] = "input";
static const char STRING_INPUT1[] = "input1";
static const char STRING_INPUT2[] = "input2";
static const char STRING_INPUT3[] = "input3";
static const char STRING_INPUT4[] = "input4";
static const char STRING_INTERVAL[] = "interval";
static const char STRING_INVERT[] = "invert";
static const char STRING_IP[] = "IP";
static const char STRING_LAST[] = "last";
static const char STRING_LEFT[] = "left";
static const char STRING_LIMITS[] = "limits";
static const char STRING_LOADV[] = "loadV";
static const char STRING_LOCATION[] = "location";
static const char STRING_LOCATION2[] = "location2";
static const char STRING_LOG[] = "log";
static const char STRING_LOOP[] = "loop";
static const char STRING_LOOPTO[] = "loopTo";
static const char STRING_MACRO[] = "macro";
static const char STRING_MAP[] = "map";
static const char STRING_MODE[] = "mode";
static const char STRING_NAME[] = "name";
static const char STR_NEW[] = "new";
static const char STRING_NUMPIXELS[] = "numPixels";
static const char STRING_OFFSET[] = "offset";
static const char STRING_OUTPUT[] = "output";
static const char STRING_PACKETS[] = "packets";
static const char STRING_PAN[] = "pan";
static const char STRING_PID[] = "PID";
static const char STRING_PINS[] = "pins";
static const char STRING_PITCH[] = "pitch";
static const char STRING_POLAR[] = "polar";
static const char STRING_PORT[] = "port";
static const char STRING_POSITION[] = "position";
static const char STRING_POWER[] = "power";
static const char STRING_PRESSURE[] = "pressure";
static const char STRING_PUBLISH[] = "publish";
static const char STRING_PUBLISHRATE[] = "publishRate";
static const char STRING_PWM_CHANNEL[] = "PWMChannel";
static const char STRING_RADIUS[] = "radius";
static const char STRING_RAW[] = "raw";
static const char STRING_RESET[] = "reset";
static const char STRING_RESETCOUNT[] = "resetCount";
static const char STRING_RIGHT[] = "right";
static const char STRING_ROLL[] = "roll";
static const char STRING_RSSI[] = "RSSI";
static const char STRING_RUDDER[] = "rudder";
static const char STRING_SAMPLES[] = "samples";
static const char STRING_SAMPLEINTERVAL[] = "sampleInterval";
static const char STRING_SATELLITES[] = "satellites";
static const char STRING_SAVE[] = "save";
static const char STRING_SCENE[] = "scene";
static const char STRING_SCENES[] = "scenes";
static const char STRING_SELECT[] = "select";
static const char STRING_SERVER[] = "server";
static const char STRING_SHEET[] = "sheet";
static const char STRING_SHUNT[] = "shunt";
static const char STRING_SHUNTV[] = "shuntV";
static const char STRING_SLEEP[] = "sleep";
static const char STRING_SOG[] = "SOG";
static const char STRING_SPEED[] = "speed";
static const char STRING_STATS[] = "stats";
static const char STRING_STATUS[] = "status";
static const char STRING_SUBSCRIBETO[] = "subs";
static const char STRING_SUB1[] = "sub1";
static const char STRING_SUB2[] = "sub2";
static const char STRING_SUB3[] = "sub3";
static const char STRING_SUB4[] = "sub4";
static const char STRING_SWITCH[] = "switch";
static const char STRING_TARGET[] = "target";
static const char STRING_TARGET_LOCATION[] = "targetLocation";
static const char STRING_TELEMETRY[] = "telemetry";
static const char STRING_TEMPERATURE[] = "temperature";
static const char STRING_THRESHOLD[] = "threshold";
static const char STRING_TIMEOUT[] = "timeout";
static const char STRING_TRIM[] = "trim";
static const char STRING_TRUE[] = "true";
static const char STRING_TURN_RATE[] = "turnRate";
static const char STRING_TYPE[] = "type";
static const char STRING_UPTIME[] = "uptime";
static const char STRING_URL[] = "URL";
static const char STRING_USAGE[] = "usage";
static const char STRING_VALUE1[] = "value1";
static const char STRING_VALUE2[] = "value2";
static const char STRING_VALUE3[] = "value3";
static const char STRING_VALUE4[] = "value4";
static const char STRING_VALUE5[] = "value5";
static const char STRING_VALUE6[] = "value6";
static const char STRING_VECTOR[] = "vector";
static const char STRING_VESSEL[] = "vessel";
static const char STRING_WAYPOINTS[] = "waypoints";
static const char STRING_WAYPOINT[] = "waypoint";
static const char STRING_WIFI[] = "wifi";
static const char STRING_WIND[] = "wind";
static const char STRING_WIND_SPEED[] = "windSpeed";
static const char STRING_WING[] = "wing";
static const char STRING_XAXIS[] = "xAxis";
static const char STRING_YAXIS[] = "yAxis";
static const char STRING_ZAXIS[] = "zAxis";





#endif
