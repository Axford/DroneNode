
useful stuff: https://www.movable-type.co.uk/scripts/latlong.html


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
