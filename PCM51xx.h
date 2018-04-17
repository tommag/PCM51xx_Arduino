/*
  MIT License

  Copyright (c) 2018 Tom Magnier

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */

#ifndef PCM51XX_H
#define PCM51XX_H

#include <Arduino.h>
#include <Wire.h>

class PCM51xx {
public:
  /* Supported sampling rates */
  enum SamplingRate {
    SAMPLE_RATE_8K        = 8000,
    SAMPLE_RATE_11_025K   = 11025,
    SAMPLE_RATE_16K       = 16000,
    SAMPLE_RATE_22_05K    = 22050,
    SAMPLE_RATE_32K       = 32000,
    SAMPLE_RATE_44_1K     = 44100,
    SAMPLE_RATE_48K       = 48000,
    SAMPLE_RATE_96K       = 96000,
    SAMPLE_RATE_192K      = 192000,
    SAMPLE_RATE_384K      = 384000
  };

  /* Supported bit depth (bits per sample) */
  enum BitDepth {
    BITS_PER_SAMPLE_16    = 16,
    BITS_PER_SAMPLE_24    = 24,
    BITS_PER_SAMPLE_32    = 32,
  };

  /* Power modes */
  enum PowerMode {
    POWER_MODE_ACTIVE     = 0x00,
    POWER_MODE_STANDBY    = 0x10,
    POWER_MODE_POWERDOWN  = 0x01
  };

  /* Power states */
  enum PowerState {
    POWER_STATE_POWERDOWN   = 0x00,
    POWER_STATE_WAIT_FOR_CP = 0x01,
    POWER_STATE_CALIB_1     = 0x02,
    POWER_STATE_CALIB_2     = 0x03,
    POWER_STATE_VOL_RAMP_UP = 0x04,
    POWER_STATE_RUN         = 0x05,
    POWER_STATE_LINE_SHORT  = 0x06,
    POWER_STATE_VOL_RAMP_DN = 0x07,
    POWER_STATE_STANDBY     = 0x08,
    POWER_STATE_I2C_FAILURE = 0x0F
  };

  /* Register addresses. Set as 16-bit values (the high byte sets the page number) */
  enum Register {
    /* Page 0 */
    RESET                 = 0x0000 + 1,   //Reset
    STANDBY_POWERDOWN     = 0x0000 + 2,   //Standby, Powerdown requests
    MUTE                  = 0x0000 + 3,   //Mute
    PLL_LOCK_PLL_ENABLE   = 0x0000 + 4,   //PLL Lock Flag, PLL enable
    SPI_MISO_FUNCTION     = 0x0000 + 6,   //SPI MISO function select
    DEEMP_EN_SDOUT_SEL    = 0x0000 + 7,   //De-emphasis enable, SDOUT select
    GPIO_EN               = 0x0000 + 8,   //GPIO enables
    BCK_LRCK_CONFIG       = 0x0000 + 9,   //BCK, LRCLK configuration
    DSG_GPIO_INPUT        = 0x0000 + 10,  //DSP GPIO Input
    MASTER_BCK_LRCK_RESET = 0x0000 + 12,  //Master mode BCK, LRCLK reset
    PLL_CLOCK_SOURCE      = 0x0000 + 13,  //PLL Clock source selection
    DAC_CLOCK_SOURCE      = 0x0000 + 14,  //DAC clock source selection
    GPIO_SRC_FOR_PLL      = 0x0000 + 18,  //GPIO source for PLL reference
    CLOCK_SYNC_REQUEST    = 0x0000 + 19,  //C
    PLL_P                 = 0x0000 + 20,  //PLL divider P factor
    PLL_J                 = 0x0000 + 21,  //PLL J part of the overall PLL multiplication factor J.D * R
    PLL_D_MSB             = 0x0000 + 22,  //PLL D part (MSB) of the overall PLL multiplication factor J.D * R
    PLL_D_LSB             = 0x0000 + 23,  //PLL D part (LSB) of the overall PLL multiplication factor J.D * R
    PLL_R                 = 0x0000 + 24,  //PLL R part of the overall PLL multiplication factor J.D * R
    DSP_CLOCK_DIV         = 0x0000 + 27,  //DSP clock divider
    DAC_CLOCK_DIV         = 0x0000 + 28,  //DAC clock divider
    NCP_CLOCK_DIV         = 0x0000 + 29,  //CP clock divider
    OSR_CLOCK_DIV         = 0x0000 + 30,  //OSR clock divider
    MASTER_BCK_DIV        = 0x0000 + 32,  //Master mode BCK divider
    MASTER_LRCK_DIV       = 0x0000 + 33,  //Master mode LRCK divider
    FS_SPEED_MODE         = 0x0000 + 34,  //16x interpolation, FS speed mode
    IDAC_MSB              = 0x0000 + 35,  //IDAC MSB (number of DSP clock cycles available in one audio frame)
    IDAC_LSB              = 0x0000 + 36,  //IDAC LSB (number of DSP clock cycles available in one audio frame)
    IGNORE_ERRORS         = 0x0000 + 37,  //Ignore various errors
    I2S_FORMAT            = 0x0000 + 40,  //I2S data format, word length
    I2S_SHIFT             = 0x0000 + 41,  //I2S shift
    DAC_DATA_PATH         = 0x0000 + 42,  //DAC data path
    DSP_PROGRAM_SEL       = 0x0000 + 43,  //DSP program selection
    CLK_MISSING_DETECTION = 0x0000 + 44,  //Clock missing detection period
    AUTO_MUTE_TIME        = 0x0000 + 59,  //Auto mute time
    DIGITAL_VOLUME_CTRL   = 0x0000 + 60,  //Digital volume control
    DIGITAL_VOLUME_L      = 0x0000 + 61,  //Digital volume for left channel
    DIGITAL_VOLUME_R      = 0x0000 + 62,  //Digital volume for right channel
    DIGITAL_VOLUME_RAMP   = 0x0000 + 63,  //Digital volume ramp up/down behavior
    DIGITAL_VOLUME_EMERG  = 0x0000 + 64,  //Digital volume emergency ramp down behavior
    AUTO_MUTE             = 0x0000 + 65,  //Auto mute
    GPIOn_OUTPUT_SEL      = 0x0000 + 80,  //GPIOn output selection. GPIO1 - GPIO6 are available (80 - 85)
    GPIO_OUTPUT_CTRL      = 0x0000 + 86,  //GPIO output value
    GPIO_OUTPUT_INVERT    = 0x0000 + 87,  //GPIO output inversion
    DSP_OVERFLOW          = 0x0000 + 90,  //DSP overflow status
    DET_FS_SCK_RATIO      = 0x0000 + 91,  //Detected FS and SCK ratio
    DET_BCK_RATIO_MSB     = 0x0000 + 92,  //Detected BCK ratio (MSB)
    DET_BCK_RATIO_LSB     = 0x0000 + 93,  //Detected BCK ratio (LSB)
    CLOCK_STATUS          = 0x0000 + 94,  //Various clock and sample rate status
    CLOCK_ERROR           = 0x0000 + 95,  //Critical clock failures reporting
    ANALOG_MUTE_MON       = 0x0000 + 108, //Analog mute monitor
    SHORT_DETECT_MON      = 0x0000 + 109, //Line output short monitor
    XSMUTE_MON            = 0x0000 + 114, //XSMUTE pin level monitor
    FS_SPEED_MODE_MON     = 0x0000 + 115, //FS speed mode monitor
    DSP_BOOT_POWER_STATE  = 0x0000 + 118, //DSP boot done flag & power state
    GPIO_INPUT            = 0x0000 + 119, //GPIO input
    AUTO_MUTE_FLAGS       = 0x0000 + 120, //Auto Mute flags

    /* Page 1 */
    OUTPUT_AMPL_TYPE      = 0x0100 + 1,   //Output amplitude type
    ANALOG_GAIN_CTRL      = 0x0100 + 2,   //Analog gain control
    UV_PROTECTION         = 0x0100 + 5,   //Undervoltage protection
    ANALOG_MUTE_CTRL      = 0x0100 + 6,   //Analog mute control
    ANALOG_GAIN_BOOST     = 0x0100 + 7,   //Analog gain boost
    VCOM_RAMP_SPEED       = 0x0100 + 8,   //VCOM ref ramp up speed
    VCOM_POWERDOWN        = 0x0100 + 9,   //VCOM powerdown control

    /* Page 44 */
    CRAM_CONTROL          = 0x2C00 + 1,   //Coefficient memory (CRAM) control

    /* Page 253 */
    CLOCK_FLEX_1          = 0xFD00 + 63,  //Clock flex register #1
    CLOCK_FLEX_2          = 0xFD00 + 64,  //Clock flex register #2
  };

  /* Constructor
   * Parameters :
   *  * The TwoWire class to use which must be initialized externally
   *  (i.e. call Wire.begin() before begin())
   *  * the I2C address to use (0x4C to 0x4F depending on the ADR1, ADR2 pins)
   */
  PCM51xx(TwoWire& wire = Wire, uint8_t i2cAddr = 0x4C);

  /* Start operation in 4-Wire I2S mode (SCK supplied to the IC)
   * Parameters :
   *  * Bits per sample (BITS_PER_SAMPLE_16 to BITS_PER_SAMPLE_32)
   *
   * Return : true if startup succeeded, false otherwise (I2C communication problem,
   * clock configuration not supported, etc)
   */
  bool begin(BitDepth bps);

  /* Start operation in 3-Wire PCM mode (SCK derived from BCK clock)
   * Parameters :
   *  * Sampling rate (SAMPLE_RATE_8K to SAMPLE_RATE_384K)
   *  * Bits per sample (BITS_PER_SAMPLE_16 to BITS_PER_SAMPLE_32)
   *
   * 2 channels are assumed.
   * Not all combinations are supported : for proper PLL operation, the BCK clock
   * (sampling rate * bits per sample * nb of channels) must be at least 1MHz and
   * must not exceed 50MHz.
   * At 44.1kHz and 48kHz only 16 and 32 bits are supported.
   *
   * Recommended minimum parameters : 32kHz, 16bit or 16kHz, 32bit
   *
   * See PM5141 datasheet table 132 for the full list of supported combinations
   * (fs = sampling rate, RSCK = 2*bit depth)
   *
   * Return : true if startup succeeded, false otherwise (I2C communication problem,
   * clock configuration not supported, etc)
   */
  bool begin(SamplingRate rate, BitDepth bps);

  /* Set power mode : active, standby, powerdown */
  void setPowerMode(PowerMode mode);

  /* Reset internal modules and registers */
  void reset();

  /* Get current power state */
  PowerState getPowerState();

  /* Set volume for left and right channels.
   * 0 : loudest (+24dB)
   * 1 : +23.5dB
   * ...
   * 254 : most quiet (-103dB)
   * 255 : mute
   */
  void setVolume(uint8_t vol);

  //TODO mute control
  //TODO DSP control (program selection, set coefficients)

  /* Write a single register */
  void writeRegister(Register address, uint8_t data);

  /* Write multiple contiguous registers */
  void writeRegisters(Register startAddress, uint8_t *data, uint8_t len);

  /* Read a single register */
  uint8_t readRegister(Register address);

  /* Read multiple contiguous registers and return the number of bytes read */
  uint8_t readRegisters(Register startAddress, uint8_t *buffer, uint8_t len);

  /* Get configured I2C address */
  uint8_t getI2CAddr() { return _i2cAddr; }

private:
  static constexpr uint8_t REG_AUTO_INCREMENT_EN = 0x80; //Auto-increment mode for multiple writes / reads

  TwoWire *_wire;
  uint8_t _i2cAddr;
  uint8_t _currentPage = 0;

  /* Called when reading/writing a register to select the correct page */
  void selectPage(Register address);
};

#endif //PCM51XX_H
