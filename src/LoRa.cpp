// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <LoRa.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_STAT           0x18
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP           0x00
#define MODE_STDBY           0x01
#define MODE_TX              0x03
#define MODE_RX_CONTINUOUS   0x05
#define MODE_RX_SINGLE       0x06

// PA config
#define PA_BOOST 0x80

// DIO mapping
#define DIO0_RX_DONE      0x00
#define DIO0_TX_DONE      0x40
#define DIO3_VALID_HEADER 0x01

// IRQ masks
#define MASK_IRQ_TX_DONE           0x08
#define MASK_IRQ_VALID_HEADER      0x10
#define MASK_IRQ_PAYLOAD_CRC_ERROR 0x20
#define MASK_IRQ_RX_DONE           0x40

// modem status masks
#define MASK_MODEM_VALID_SIGNAL 0x01
#define MASK_MODEM_SYNCED       0x02
#define MASK_MODEM_RX_ACTIVE    0x04
#define MASK_MODEM_VALID_HEADER 0x08
#define MASK_MODEM_CLEAR        0x10

// RF
#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT   157
#define RSSI_OFFSET_LF_PORT   164

#define MAX_PKT_LENGTH 255

#if (ESP8266 || ESP32)
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

//******************************************************************************
// Public Functions
//******************************************************************************

LoRaClass::LoRaClass() :
    _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
    _spi(&LORA_DEFAULT_SPI),
    _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
    _frequency(0),
    _implicitHeaderMode(0),
    _transmitting(false),
    _packetIndex(0),
    _available(0),
    _onValidHeader(NULL),
    _onRxDone(NULL),
    _onTxDone(NULL)
{
    // overide Stream timeout value
    setTimeout(0);
}

void LoRaClass::setPins(uint8_t ss, uint8_t reset, uint8_t dio0)
{
    _ss = ss;
    _reset = reset;
    _dio0 = dio0;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

bool LoRaClass::begin(long frequency, uint8_t txPower)
{
    // setup pins
    pinMode(_ss, OUTPUT);
    pinMode(_dio0, INPUT);

    // set SS high
    digitalWrite(_ss, HIGH);

    if (_reset != -1)
    {
        pinMode(_reset, OUTPUT);

        // perform reset
        digitalWrite(_reset, LOW);
        delay(10);
        digitalWrite(_reset, HIGH);
        delay(10);
    }

    // start SPI
    _spi->begin();

    // check version
    if (readRegister(REG_VERSION) != 0x12)
        return 0;

    // put in sleep mode
    sleep();

    // set frequency
    setFrequency(frequency);

    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // disable unused IRQs
    writeRegister(REG_IRQ_FLAGS_MASK, ~(MASK_IRQ_TX_DONE |
                                        MASK_IRQ_VALID_HEADER |
                                        MASK_IRQ_PAYLOAD_CRC_ERROR |
                                        MASK_IRQ_RX_DONE));

    // set LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power
    setTxPower(txPower);

    // put in standby mode
    idle();

    // attach ISR
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);

    return 1;
}

void LoRaClass::end()
{
    // put in sleep mode
    sleep();

    // stop SPI
    _spi->end();
}

void LoRaClass::setTxPower(uint8_t level, uint8_t outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin)
    {
        if (level > 14)
            level = 14;

        writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else // PA BOOST
    {
        if (level > 17)
        {
            if (level > 20)
                level = 20;

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        }
        else // level <= 17
        {
            if (level < 2)
                level = 2;

            //Default value PA_HF/LF or +17dBm
            writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }

        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRaClass::setFrequency(long frequency)
{
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRaClass::setSpreadingFactor(uint8_t sf)
{
    if (sf < 6)
        sf = 6;
    else if (sf > 12)
        sf = 12;

    if (sf == 6)
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    setLdoFlag();
}

void LoRaClass::setSignalBandwidth(long sbw)
{
    uint8_t bw;

    if (sbw <= 7.8E3)
        bw = 0;
    else if (sbw <= 10.4E3)
        bw = 1;
    else if (sbw <= 15.6E3)
        bw = 2;
    else if (sbw <= 20.8E3)
        bw = 3;
    else if (sbw <= 31.25E3)
        bw = 4;
    else if (sbw <= 41.7E3)
        bw = 5;
    else if (sbw <= 62.5E3)
        bw = 6;
    else if (sbw <= 125E3)
        bw = 7;
    else if (sbw <= 250E3)
        bw = 8;
    else // 500E3
        bw = 9;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    setLdoFlag();
}

void LoRaClass::setCodingRate4(uint8_t denominator)
{
    if (denominator < 5)
        denominator = 5;
    else if (denominator > 8)
        denominator = 8;

    uint8_t cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(uint16_t length)
{
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(uint8_t sw)
{
    writeRegister(REG_SYNC_WORD, sw);
}

void LoRaClass::enableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableInvertIQ()
{
    writeRegister(REG_INVERTIQ,  0x66);
    writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ()
{
    writeRegister(REG_INVERTIQ,  0x27);
    writeRegister(REG_INVERTIQ2, 0x1d);
}

void LoRaClass::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120)
        ocpTrim = (mA - 45) / 5;
    else if (mA <=240)
        ocpTrim = (mA + 30) / 10;

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaClass::setGain(uint8_t gain)
{
    if (gain > 6)
        gain = 6;

    // set to standby
    idle();

    if (gain == 0)
    {
        // if gain = 0, enable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x04); // TODO use bit set
    }
    else
    {
        // disable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x00); // TODO use bit set

        // set gain and LNA boost
        writeRegister(REG_LNA, (gain << 5) | 0x03);
    }
}

void LoRaClass::continuousRx(uint8_t size)
{
    // wait for any existing transmission to complete
    while (_transmitting)
        yield();

    writeRegister(REG_DIO_MAPPING_1, DIO0_RX_DONE | DIO3_VALID_HEADER);

    if (size > 0)
    {
        implicitHeaderMode();
        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
        explicitHeaderMode();

    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRaClass::singleRx()
{
    if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // wait for any existing transmission to complete
        while (_transmitting)
            yield();

        // reset FIFO address
        resetFifo();

        writeRegister(REG_DIO_MAPPING_1, DIO0_RX_DONE | DIO3_VALID_HEADER);

        // put in single RX mode
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
}

void LoRaClass::idle()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::beginPacket(int implicitHeader)
{
    // wait for any existing transmission to complete
    while (_transmitting)
        yield();

    // put in standby mode
    idle();

    if (implicitHeader)
        implicitHeaderMode();
    else
        explicitHeaderMode();

    // reset FIFO address and paload length
    resetFifo();
    writeRegister(REG_PAYLOAD_LENGTH, 0);
}

size_t LoRaClass::write(uint8_t byte)
{
    return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
    uint8_t currentLength = readRegister(REG_PAYLOAD_LENGTH);

    // check size
    if ((size + currentLength) > MAX_PKT_LENGTH)
        size = MAX_PKT_LENGTH - currentLength;

    // write data
    for (uint8_t i = 0; i < size; i++)
        writeRegister(REG_FIFO, buffer[i]);

    // update length
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

void LoRaClass::endPacket(bool blocking)
{
    writeRegister(REG_DIO_MAPPING_1, DIO0_TX_DONE | DIO3_VALID_HEADER);

    // put in TX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    _transmitting = true;

    if (blocking)
        while (_transmitting)
            yield();
}

int LoRaClass::available()
{
    if (_available > _packetIndex)
        return _available - _packetIndex;
    return 0;
}

int LoRaClass::read()
{
    if (!available())
        return -1;

    _packetIndex++;

    return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
    if (!available())
        return -1;

    // store current FIFO address
    uint8_t currentAddress = readRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = readRegister(REG_FIFO);

    // restore FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

int LoRaClass::packetRssi()
{
    return readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT);
}

float LoRaClass::packetSnr()
{
    return (int8_t)readRegister(REG_PKT_SNR_VALUE) * 0.25f;
}

long LoRaClass::packetFrequencyError()
{
    int32_t freqError = 0;
    freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & B111);
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

    if (readRegister(REG_FREQ_ERROR_MSB) & B1000) // Sign bit is on
        freqError -= 524288; // B1000'0000'0000'0000'0000

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

    return static_cast<long>(fError);
}

bool LoRaClass::validSignalDetected()
{
    return readRegister(REG_MODEM_STAT) & MASK_MODEM_VALID_SIGNAL;
}

int LoRaClass::rssi()
{
    return readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT);
}

uint8_t LoRaClass::random()
{
    return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaClass::onValidHeader(void(*callback)(), uint8_t dio3)
{
    _onValidHeader = callback;

    if (_onValidHeader) // setup DIO3 and attach ISR
    {
        pinMode(dio3, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.usingInterrupt(digitalPinToInterrupt(dio3));
#endif
        attachInterrupt(digitalPinToInterrupt(dio3), LoRaClass::onDio3Rise, RISING);
    }
    else // detach ISR
    {
        detachInterrupt(digitalPinToInterrupt(dio3));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(dio3));
#endif
    }
}

void LoRaClass::dumpRegisters(Stream& out)
{
    for (uint8_t i = 0; i < 128; i++)
    {
        out.print("0x");
        out.print(i, HEX);
        out.print(": 0x");
        out.println(readRegister(i), HEX);
    }
}

void LoRaClass::debug()
{
    Serial.print("mode: "); Serial.print(readRegister(REG_OP_MODE), BIN); Serial.print(" status: "); Serial.println(readRegister(REG_MODEM_STAT), BIN);
}

//******************************************************************************
// Private Functions
//******************************************************************************

uint8_t LoRaClass::getSpreadingFactor()
{
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

long LoRaClass::getSignalBandwidth()
{
    uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (bw)
    {
        case 0: return 7.8E3;
        case 1: return 10.4E3;
        case 2: return 15.6E3;
        case 3: return 20.8E3;
        case 4: return 31.25E3;
        case 5: return 41.7E3;
        case 6: return 62.5E3;
        case 7: return 125E3;
        case 8: return 250E3;
        case 9: return 500E3;
    }
    return -1;
}

void LoRaClass::explicitHeaderMode()
{
    _implicitHeaderMode = 0;
    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
    _implicitHeaderMode = 1;
    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::setLdoFlag()
{
    // Section 4.1.1.5
    float symbolDuration = 1000.0f / ((float)getSignalBandwidth() / (1u << getSpreadingFactor()));
    Serial.print("symbolDuration: "); Serial.println(symbolDuration);

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16.0f;

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRaClass::resetFifo()
{
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    _available = 0;
    _packetIndex = 0;
}

ISR_PREFIX void LoRaClass::onDio0Rise()
{
    LoRa.handleDio0Rise();
}

void LoRaClass::handleDio0Rise()
{
    uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
    // Serial.print("---- IRQ: "); Serial.println(irqFlags, BIN);

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, MASK_IRQ_TX_DONE | MASK_IRQ_PAYLOAD_CRC_ERROR | MASK_IRQ_RX_DONE);

    if (irqFlags & MASK_IRQ_TX_DONE)
    {
        _transmitting = false;
        if (_onTxDone)
            _onTxDone();
    }
    else if ((irqFlags & MASK_IRQ_RX_DONE) && !(irqFlags & MASK_IRQ_PAYLOAD_CRC_ERROR))
    {
        // received a packet
        _packetIndex = 0;

        // read packet length
        _available = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        if (_onRxDone)
            _onRxDone(_available);
    }
}

ISR_PREFIX void LoRaClass::onDio3Rise()
{
    LoRa.handleDio3Rise();
}

void LoRaClass::handleDio3Rise()
{
    uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
    // Serial.print("---- IRQ: "); Serial.println(irqFlags, BIN);

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, MASK_IRQ_VALID_HEADER);

    if (irqFlags & MASK_IRQ_VALID_HEADER)
    {
        if (_onValidHeader)
            _onValidHeader();
    }
}

uint8_t LoRaClass::readRegister(uint8_t address)
{
    return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
    singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;

    digitalWrite(_ss, LOW);

    _spi->beginTransaction(_spiSettings);
    _spi->transfer(address);
    response = _spi->transfer(value);
    _spi->endTransaction();

    digitalWrite(_ss, HIGH);

    return response;
}

LoRaClass LoRa;
