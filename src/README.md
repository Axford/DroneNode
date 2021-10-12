
# todo

Controller
* AP Config - switch between AP mode and STA mode on the fly
* Possibly choose between several pre-configured wireless networks

Neopixel module
* WS2182b - maybe add config for different LED types?
* Variable number of LEDs
* Assign them to bow, stern, port, stardboard?   
*  or just assign colour values per LED
* Brightness control
* Turn on/off
* Mode input (e.g. from a switch module or battery alarm)

Neopixel Module
* Disable module will clear LEDs first (turn off) -so can be used to toggle on/off
* Base settings:
  - Pin
  - Num pixels (automatically divided into four segments)
  - Colour order (e.g. GRB vs RGB)
  - Driver type/speed (e.g. NEO_KHZ800)
* Subs
  - Scene number (0..x) - so other modules can select a scene
  - Active scene - so other modules can configure the active scene - for extensibility
* Params
  - 0..x scenes - with addresses incrementing from xx (like waypoints)

* Scene (struct stored in uint8_t msg):
  - brightness - overall brightness for this scene
  - effect:
      0: solid Colour
      1: flash
      2: theatre chase
      3: breath
  - effect param 1:
      e.g. flash speed
  - effect param 2:
      e.g. flash duty cycle
  - segment 1..4 - r,g,b (12 bytes total)



* add a physical pot on the controller for trim, etc
* Speed limiter - add as limits to motor module, similar to servo
* FIX the LAG
   * Add a lag monitoring system... ping? part of mgmt module?
   * Turn off discovery dynamically (from controller)
   * Prioritise specifc channel subscriptions over wildcards
* Additional mgmt controls on controller ... like reset?
   * Turn off remote UDP when out of range


useful stuff: https://www.movable-type.co.uk/scripts/latlong.html


* convert all inputs to addr params - mostly done
* allow easy editing of input params via UI
* explore a node-based visualisation of param wiring


Switch module
* can switch between two or more inputs based on a control value
eg. to switch between waypoint locations and a home location based on low voltage trigger


* trigger firmware update via mamangement module


waypoint nav
* visualise target corridor
* change radius
* online adding/removing waypoints
* move waypoints to separate config file
* save changes to waypoint file


servo
* add centre calibration value

motor + rudder controller
* Sub to target heading
* Sub to current heading
* Sub to distance
* Generate rudder and motor speed values
* add rudder range
* implement PID for rudder control vs current heading


* Modules to implement:
  * magnetic angle sensor
  * PID Controller (e.g. to wire up an angle sensor and motor driver to make a servo)
  * AIS - AIS parser for GPS - https://github.com/KimBP/AIS/
    * what todo with decoded messages?  pub somewhere?


# libraries

## LinkedList
3rd party library for generic linked list


## DroneLinkMsg
Messages in binary format
Struct: DRONE_LINK_MSG
class: DroneLinkMsg

## DroneLinkChannel
Manages a single pub/sub channel.  Maintains a list of subscribers
Setup:
 - Set Channel ID

## DroneLinkManager
Manages channels and allows for single point of interaction for modules.
Channel objects are created on demand as subs register
provides the master pub function that all modules use


## DroneModule
Base class for modules.
At creation:
* Passed a DroneModuleManager - stored and used to register itself
* Passed a DroneLinkManager - stored and used to wire up subs.


## DroneModuleManager
Manages all active modules.  Deals with the management channel (e.g. dynamic sub changes, activate/deactivate modules)


# Setup

* Create DroneLinkManager
* Create modules... passing manager object
  * Modules register subs, which in turn creates Channels as needed


# Modules

* telemetry
On receive of telemetry messages, publishes them via DroneLinkManager.  
Is subscribed to anything that should be transmitted on change.  
