ESP8266 / SPH0645
=================

I2S Interface for ESP8266 and SPH0645 MEMS microphone.

Reads data from the microphone and outputs to the Serial Plotter.
There are some mysterious calibration factors in there, but this
outputs floating point values from -1 to +1.

Requirements
------------

See https://github.com/esp8266/Arduino

Upload
------

To compile and upload to device

    make flash
