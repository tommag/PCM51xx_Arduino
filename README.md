# PCM51xx_Arduino
Arduino library for TI PCM51xx DAC ICs software configuration via I2C.

The following ICs are compatible and supported :
* PCM5121 / PCM5122
* PCM5141 / PCM5142

The PCM51x2 devices have a better SNR than the PCM51x1.

The PCM514x have a full featured DSP whereas the PCM512x only supports a fixed processing flow. Otherwise they are identical.

The PCM51xx can be used in hardware or software configuration mode. To set software configuration mode, set the pins MODE1 and MODE2 to select I2C / SPI mode. This library only supports I2C mode at the moment.

Four different I2C addresses can be selected using the ADR1 and ADR2 pins (0x4C to 0x4F). The library defaults to 0x4C (ADR1 and ADR2 low).

## Usage
See the examples.

The Wire library must be initialized before calling `PCM51xx::begin()`.

If 3-wire I2S is used (i.e. the DAC system clock is derived from the I2S bit clock), the I2S source should be started before configuring the clock.
