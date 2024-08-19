// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>

#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6 
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

class LoRaClass : public Stream {
public:
  LoRaClass();

  bool begin(long frequency);
  void end();

  bool beginPacket(int implicitHeader = false);
  bool endPacket(bool async = false);

  uint8_t parsePacket(uint8_t size = 0);
  int packetRssi();
  float packetSnr();
  long packetFrequencyError();

  int rssi();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(uint8_t));
  void onTxDone(void(*callback)());

  void continuousRx(uint8_t size = 0);
  void singleRx();
  void idle();
  void sleep();

  void setTxPower(uint8_t level, uint8_t outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(uint8_t sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(uint8_t denominator);
  void setPreambleLength(uint16_t length);
  void setSyncWord(uint8_t sw);
  void enableCrc();
  void disableCrc();
  void enableInvertIQ();
  void disableInvertIQ();
  
  void setOCP(uint8_t mA); // Over Current Protection control
  
  void setGain(uint8_t gain); // Set LNA gain

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  uint8_t random();

  void setPins(uint8_t ss = LORA_DEFAULT_SS_PIN, uint8_t reset = LORA_DEFAULT_RESET_PIN, uint8_t dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPI(SPIClass& spi);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);
  void debug();

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();
  // bool isTransmitting();
  void resetFifo();

  uint8_t getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();

private:
  SPISettings _spiSettings;
  SPIClass* _spi;
  uint8_t _ss;
  uint8_t _reset;
  uint8_t _dio0;
  long _frequency;
  bool _implicitHeaderMode;
  volatile bool _transmitting;
  volatile uint8_t _packetIndex;
  volatile uint8_t _available;
  void (*_onReceive)(uint8_t);
  void (*_onTxDone)();
};

extern LoRaClass LoRa;

#endif
