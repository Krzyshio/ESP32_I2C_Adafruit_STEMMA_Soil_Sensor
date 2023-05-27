# ESP32 I2C Adafruit STEMMA Soil Sensor Library for ESP-IDF

## Introduction

This component provides access to the Adafruit STEMMA Soil Sensor, a capacitive moisture sensor with temperature capabilities. The project is designed for the ESP-IDF environment.

The creation of this library arose from a need for a dedicated solution for the ESP-IDF platform, as no such solution existed at the time of development.

It is written and tested for the [ESP-IDF](https://github.com/espressif/esp-idf) environment.

## Dependencies

The ESP-IDF environment is required to use this library.

## Example

An example of how to use this library will be added in the future.

## Features

* Allows for the reading of moisture levels and temperature from the Adafruit STEMMA Soil Sensor.
* Supports ESP32 I2C interface for sensor communication.
* Works with multiple sensors (set unique I2C addresses for each device).
* Offers static allocation of the device instance with no global variables.

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

* [Adafruit STEMMA Soil Sensor](https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor)
* [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)

## Acknowledgements

* The project is built for the ESP-IDF environment developed by Espressif.