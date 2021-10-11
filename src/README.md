
# todo

* add a physical pot on the controller for trim, etc
* Speed limiter
* SNR from radio?
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

other
* auto-generate base config if it doesn't exist
* add wifi manager

UI stuff for setting up a new node
* set config via UI
  * define config template per module type
  * copy config from another node via UI


* trigger firmware update via mamangement module



waypoint nav
* visualise target corridor
* change radius
* online adding/removing waypoints
* move waypoints to separate config file
* save changes to waypoint file


servo
* add centre calibration value
* add limits

motor + rudder controller
* Sub to target heading
* Sub to current heading
* Sub to distance
* Generate rudder and motor speed values
* add rudder range
* implement PID for rudder control vs current heading


add telemetry interface to server
* TRANSMIT: fix telemetry transmit - doesn't seem to be receiving properly


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
