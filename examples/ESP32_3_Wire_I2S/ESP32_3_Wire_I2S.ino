/* PCM51xx library example, using the ESP8266Audio "PlayRTTTLToI2SDAC" example
 *
 * Tested on an ESP32 board.
 *
 * Don't forget to set the correct pinout for the I2S and I2C peripherals for
 * your hardware.
 *
 * You have to change the rate to 44100 in AudioGeneratorRTTTL.cpp to output
 * the correct bit clock.
 */

//ESP8266Audio includes
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorRTTTL.h"
#include "AudioOutputI2S.h"

#include "PCM51xx.h"
#include "Wire.h"

const char rudolph[] PROGMEM =
"Rudolph the Red Nosed Raindeer:d=8,o=5,b=250:g,4a,g,4e,4c6,4a,2g.,g,a,g,a,4g,4c6,2b.,4p,f,4g,f,4d,4b,4a,2g.,g,a,g,a,4g,4a,2e.,4p,g,4a,a,4e,4c6,4a,2g.,g,a,g,a,4g,4c6,2b.,4p,f,4g,f,4d,4b,4a,2g.,g,a,g,a,4g,4d6,2c.6,4p,4a,4a,4c6,4a,4g,4e,2g,4d,4e,4g,4a,4b,4b,2b,4c6,4c6,4b,4a,4g,4f,2d,g,4a,g,4e,4c6,4a,2g.,g,a,g,a,4g,4c6,2b.,4p,f,4g,f,4d,4b,4a,2g.,4g,4a,4g,4a,2g,2d6,1c.6.";
// Plenty more at: http://mines.lumpylumpy.com/Electronics/Computers/Software/Cpp/MFC/RingTones.RTTTL

AudioGeneratorRTTTL *rtttl;
AudioFileSourcePROGMEM *file;
AudioOutputI2S *out;

PCM51xx pcm(Wire); //Using the default I2C address 0x74

void setup()
{
  Serial.begin(115200);
  delay(1000);

  out = new AudioOutputI2S();
  out->SetPinout(25, 27, 26); //HW dependent ! BCK, LRCK, DATA
  out->SetGain(0.1);

  Wire.begin(2, 4); //HW dependent ! SDA, SCL

  //!\ the AudioGeneratorRTTTL must be set to 44.1kHz, 16 bits.
  // You have to change the rate to 44100 in AudioGeneratorRTTTL.cpp
  if (pcm.begin(PCM51xx::SAMPLE_RATE_44_1K, PCM51xx::BITS_PER_SAMPLE_16))
    Serial.println("PCM51xx initialized successfully.");
  else
  {
    Serial.println("Failed to initialize PCM51xx.");
    uint8_t powerState = pcm.getPowerState();
    if (powerState == PCM51xx::POWER_STATE_I2C_FAILURE)
    {
      Serial.print("No answer on I2C bus at address ");
      Serial.println(pcm.getI2CAddr());
    }
    else
    {
      Serial.print("Power state : ");
      Serial.println(pcm.getPowerState());
      Serial.println("Check that the sample rate / bit depth combination is supported.");
    }
  }

  //Set volume
  pcm.setVolume(127);

  Serial.println("RTTTL start");
  file = new AudioFileSourcePROGMEM( rudolph, strlen_P(rudolph) );

  rtttl = new AudioGeneratorRTTTL();
  rtttl->begin(file, out);
}

void loop()
{
  if (rtttl->isRunning()) {
    if (!rtttl->loop()) rtttl->stop();
  } else {
    Serial.println("RTTTL done");
    pcm.setPowerMode(PCM51xx::POWER_MODE_STANDBY);
    delay(1000);
  }
}
