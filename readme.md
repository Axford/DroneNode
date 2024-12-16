
### DroneNode

** Part of the Robot Boat Pioneers project **

Each boat (or other device) is based on a custom ESP32-based motherboard running our DroneNode firmware.  The firmware exposes a number of sensors, actuators, telemetry interfaces and control modules that can be configured for any given boat (device) through a simple config file.  To keep software modules loosely coupled, they communicate using a pub/sub messaging model. 

See this article for more information: https://robotboatpioneers.blogspot.com/2023/01/dronenode-modules-and-loosely-coupled.html


## Compiling ##

Requires Platform.io.  Tested on Mac, Windows and Linux.  Intended for an ESP32 DevKit v1 in combination with a DroneLink motherboard.  

