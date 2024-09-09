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

    // HW config
    void setPins(uint8_t ss = LORA_DEFAULT_SS_PIN,
                 uint8_t reset = LORA_DEFAULT_RESET_PIN,
                 uint8_t dio0 = LORA_DEFAULT_DIO0_PIN);
    void setSPI(SPIClass& spi) { _spi = &spi; }
    void setSPIFrequency(uint32_t frequency);

    // initiate / terminate connection with radio
    bool begin(long frequency, uint8_t txPower);
    void end();

    // radio config
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
    void setOCP(uint8_t mA);
    void setGain(uint8_t gain);

    // radio modes
    void continuousRx(uint8_t size = 0);
    void singleRx();
    void idle();
    void sleep();

    // TX
    void beginPacket(int implicitHeader = false);
    virtual size_t write(uint8_t byte);
    virtual size_t write(const uint8_t *buffer, size_t size);
    void endPacket(bool blocking = false);

    // RX
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush() {}
    int packetRssi();
    float packetSnr();
    long packetFrequencyError();
    bool validSignalDetected();

    // other functionality
    int rssi();
    uint8_t random();
    void onValidHeader(void(*callback)(), uint8_t dio3 = 0);
    void onRxDone(void(*callback)(uint8_t)) { _onRxDone = callback; }
    void onTxDone(void(*callback)()) { _onTxDone = callback; }

    // debug
    void dumpRegisters(Stream& out);
    void debug();

private:
    // get radio config
    uint8_t getSpreadingFactor();
    long getSignalBandwidth();

    // set radio config
    void explicitHeaderMode();
    void implicitHeaderMode();
    void setLdoFlag();
    void resetFifo();

    // ISR
    static void onDio0Rise();
    void handleDio0Rise();
    static void onDio3Rise();
    void handleDio3Rise();

    // SPI
    uint8_t readRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t singleTransfer(uint8_t address, uint8_t value);

    // member vars
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
    void (*_onValidHeader)();
    void (*_onRxDone)(uint8_t);
    void (*_onTxDone)();
};

extern LoRaClass LoRa;

#endif
