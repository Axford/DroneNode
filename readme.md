
### DroneNode

## DroneSystem

DroneSystem is the root of all Drone objects, keeping the main script as simple as possible.  System object is passed to all modules as means to access underlying hardware and services.

* Hosts all the other Drone core objects (module mgr, execution mgr, etc)
* HAL and underlying services
  * Status LEDs (neopixel strings)
  * interrupt routines
  * filesystem (flash vs SD)
  * timers?
  * sleep system?
  * cpu freq?
  * Logging
  * Serial ports?
  * I2C multiplexer
  * SPI conflicts?
  * memory usage
  * cpu utilisation
  * firmware OTA updates
  * serial interface (for manual config/overrides)
