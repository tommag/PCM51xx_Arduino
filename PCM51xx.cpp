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

#include "PCM51xx.h"

PCM51xx::PCM51xx(TwoWire& wire, uint8_t i2cAddr):
  _wire(&wire),
  _i2cAddr(i2cAddr)
{

}


bool PCM51xx::begin(BitDepth bps)
{
  //Force a page counter sync between the local variable and the IC
  _currentPage = 0xFF;
  selectPage(RESET);

  // Set correct I2S config
  uint8_t config = 0;
  switch (bps) {
    case BITS_PER_SAMPLE_16: config = 0x00; break;
    case BITS_PER_SAMPLE_24: config = 0x02; break;
    case BITS_PER_SAMPLE_32: config = 0x03; break;
  };
  writeRegister(I2S_FORMAT, config);

  delay(10); //Wait for calibration, startup, etc

  return getPowerState() == POWER_STATE_RUN;
}

bool PCM51xx::begin(SamplingRate rate, BitDepth bps)
{
  //See here : https://e2e.ti.com/support/data_converters/audio_converters/f/64/t/428281
  // for a config example

  // Check that the bit clock (PLL input) is between 1MHz and 50MHz
  uint32_t bckFreq = rate * bps * 2;
  if (bckFreq < 1000000 || bckFreq > 50000000)
    return false;

  // 24 bits is not supported for 44.1kHz and 48kHz.
  if ((rate == SAMPLE_RATE_44_1K || rate == SAMPLE_RATE_48K) && bps == BITS_PER_SAMPLE_24)
    return false;

  //Force a page counter sync between the local variable and the IC
  _currentPage = 0xFF;

  //Initialize system clock from the I2S BCK input
  writeRegister(IGNORE_ERRORS, 0x1A);  // Disable clock autoset and ignore SCK detection
  writeRegister(PLL_CLOCK_SOURCE, 0x10);  // Set PLL clock source to BCK
  writeRegister(DAC_CLOCK_SOURCE, 0x10); // Set DAC clock source to PLL output

  //PLL configuration
  int p, j, d, r;

  //Clock dividers
  int nmac, ndac, ncp, dosr, idac;

  if (rate == SAMPLE_RATE_11_025K || rate == SAMPLE_RATE_22_05K || rate == SAMPLE_RATE_44_1K)
  {
    //44.1kHz and derivatives.
    //P = 1, R = 2, D = 0 for all supported combinations.
    //Set J to have PLL clk = 90.3168 MHz
    p = 1;
    r = 2;
    j = 90316800 / bckFreq / r;
    d = 0;

    //Derive clocks from the 90.3168MHz PLL
    nmac = 2;
    ndac = 16;
    ncp = 4;
    dosr = 8;
    idac = 1024; // DSP clock / sample rate
  }
  else
  {
    //8kHz and multiples.
    //PLL config for a 98.304 MHz PLL clk
    if (bps == BITS_PER_SAMPLE_24 && bckFreq > 1536000)
      p = 3;
    else if (bckFreq > 12288000)
      p = 2;
    else
      p = 1;

    r = 2;
    j = 98304000 / (bckFreq / p) / r;
    d = 0;

    //Derive clocks from the 98.304MHz PLL
    switch (rate) {
      case SAMPLE_RATE_16K: nmac = 6; break;
      case SAMPLE_RATE_32K: nmac = 3; break;
      default:              nmac = 2; break;
    };

    ndac = 16;
    ncp = 4;
    dosr = 384000 / rate;
    idac = 98304000 / nmac / rate; // DSP clock / sample rate
  }


  // Configure PLL
  writeRegister(PLL_P, p - 1);
  writeRegister(PLL_J, j);
  writeRegister(PLL_D_MSB, (r >> 8) & 0x3F);
  writeRegister(PLL_D_LSB, r & 0xFF);
  writeRegister(PLL_R, r - 1);

  // Clock dividers
  writeRegister(DSP_CLOCK_DIV, nmac - 1);
  writeRegister(DAC_CLOCK_DIV, ndac - 1);
  writeRegister(NCP_CLOCK_DIV, ncp - 1);
  writeRegister(OSR_CLOCK_DIV, dosr - 1);

  // IDAC (nb of DSP clock cycles per sample)
  writeRegister(IDAC_MSB, (idac >> 8) & 0xFF);
  writeRegister(IDAC_LSB, idac & 0xFF);

  return begin(bps);
}

void PCM51xx::setPowerMode(PowerMode mode)
{
  writeRegister(STANDBY_POWERDOWN, mode);
}

void PCM51xx::reset()
{
  setPowerMode(POWER_MODE_STANDBY);
  writeRegister(RESET, 0x11);
  setPowerMode(POWER_MODE_ACTIVE);
}

PCM51xx::PowerState PCM51xx::getPowerState()
{
  uint8_t regValue = readRegister(DSP_BOOT_POWER_STATE);

  return (PowerState)(regValue & 0x0F);
}

void PCM51xx::setVolume(uint8_t vol)
{
  writeRegister(DIGITAL_VOLUME_L, vol);
  writeRegister(DIGITAL_VOLUME_R, vol);
}

void PCM51xx::writeRegister(Register address, uint8_t data)
{
  selectPage(address);

  _wire->beginTransmission(_i2cAddr);
  _wire->write(address);
  _wire->write(data);
  _wire->endTransmission();
}

void PCM51xx::writeRegisters(Register startAddress, uint8_t *data, uint8_t len)
{
  selectPage(startAddress);

  _wire->beginTransmission(_i2cAddr);
  _wire->write((uint8_t)startAddress | REG_AUTO_INCREMENT_EN);
  for (int i = 0; i < len; i++)
    _wire->write(data[i]);
  _wire->endTransmission();
}

uint8_t PCM51xx::readRegister(Register address)
{
  selectPage(address);

  _wire->beginTransmission(_i2cAddr);
  _wire->write(address);
  _wire->endTransmission();

  _wire->requestFrom(_i2cAddr, 1);
  return _wire->read();
}

uint8_t PCM51xx::readRegisters(Register startAddress, uint8_t *buffer, uint8_t len)
{
  selectPage(startAddress);

  _wire->beginTransmission(_i2cAddr);
  _wire->write((uint8_t)startAddress | REG_AUTO_INCREMENT_EN);
  _wire->endTransmission();

  _wire->requestFrom(_i2cAddr, len);
  return _wire->readBytes(buffer, len);
}

void PCM51xx::selectPage(Register address)
{
  uint8_t page = (address >> 8) & 0xFF;

  if (page != _currentPage)
  {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(0);
    _wire->write(page);
    _wire->endTransmission();

    _currentPage = page;
  }
}
