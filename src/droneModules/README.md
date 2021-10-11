

# Creating a new DroneModule

## Overview

* DroneModules inherit from the base DroneModule class (defined in src/DroneModule.h)
* Modules that wrap an I2C device can inherit from the I2CBaseModule (defined in src/droneModules/I2CBaseModule.h) - which includes a lot of the base I2C functionality, including configurable I2C bus (_bus) and address (_addr)

## Process

In DroneNode git repo:
* Copy an existing module (.h and .cpp) with similar structure, name it appropriately
* Rename and rework to suit your application
* New string constants should be added to DroneModule.h, following the existing style
* Link the new module into src/DroneModuleManager.h to allow it to be loaded:
    * Add an include for your new module to the DroneModuleManager header
    * Modify the large "if" statement in loadModulesFromJSON() to include your module - around line160)


In DroneLinkServer git repo:
* Modify /public/moduleSchema.json to add an entry for your new module
  * Define the schema for your module (see existing for examples)


To test:
* Build and install the new firmware onto a physical nodes
* Restart the server
* Open configurator for the node and add an instance of your new module
* Configure, save and restart
* TEST TEST TEST :)
