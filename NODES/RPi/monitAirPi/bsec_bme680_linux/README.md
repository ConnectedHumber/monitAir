# bsec_bme680_linux

Read the BME680 sensor with the BSEC library on Linux (e.g. Raspberry Pi)

## Intro

Working example using the
[BME680 sensor](https://www.bosch-sensortec.com/en/bst/products/all_products/bme680)
on Linux (e.g. Raspberry Pi) with the precompiled
[BSEC library](https://www.bosch-sensortec.com/bst/products/all_products/bsec),
which allows to read calibrated environment values including an actual Indoor
Air Quality (IAQ) score.

It makes use of
[Bosch's provided driver](https://github.com/BoschSensortec/BME680_driver)
and can be configured in terms of it.
Readings will be directly output to stdout in a loop.

## Prerequisites

[Download the BSEC software package from Bosch](https://www.bosch-sensortec.com/bst/products/all_products/bsec)
and unpack into `./src`, 

## Configure and Compile

Optionally make changes to make.config.

Depending on how your sensor is embedded it might be surrounded by other
components giving off heat. Use an offset in °C in `bsec_bme680.c` to
compensate. The default is 1 °C:  This is BME680 reading 1.0C higher than refrence. 
```
#define temp_offset (1.0f)
```

To compile: `./make.sh`

## Usage

Test and you should have output similar to:

```
$ ./bsec_bme680
  {IAQ_Accuracy: 1, IAQ: 25.0, Temperature: 22.4, Humidity: 76.4, Pressure: 1002.33, Gas: 240234.52, Status: 0}

```

It can easily be modified in the `output_ready` function.
IAQ_Accuracy, Gas and Status not properly used in monitAirPi.py yet.

The BSEC library is supposed to create an internal state of calibration with
increasing accuracy over time. Each 10.000 samples it will save the internal
calibration state to `./bsec_iaq.state` (or wherever you specify the config
directory to be) so it can pick up where it was after interruption.

Initial development and for more information or updates from the original github
https://github.com/alexh-name/
