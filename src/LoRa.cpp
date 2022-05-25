// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#include <LoRa.h>

/// Notes:
/// LNA - Low Noise Amplifier
/// AGC - Automatic Gain Control
/// LO - Local Oscillator

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0b
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_RSSI_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_HIGH_BW_OPTIMIZE_1 0x36
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_HIGH_BW_OPTIMIZE_2 0x3a
#define REG_INVERTIQ2 0x3b
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d
#define REG_AGC_Thresh3L_F 0x64

// DIO0 mapping Bits 7-6 of REG_DIO_MAPPING_1.
// Table 18
#define DIO0_RX_DONE 0x00  // RxDone  00 000000
#define DIO0_TX_DONE 0x40  // TDone   01 000000
#define DIO0_CAD_DONE 0x80 // CadDone 10 000000

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06
#define MODE_CAD 0x07

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_RX_TIMEOUT_MASK 0x80
#define IRQ_RX_DONE_MASK 0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_VALID_HEADER_MASK 0x10
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_CAD_DONE_MASK 0x04
#define IRQ_FHSS_CHANGE_CH_MASK 0x02
#define IRQ_CAD_DETECTED_MASK 0x01

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define MAX_PKT_LENGTH 255

#if (ESP8266 || ESP32)
#define ISR_PREFIX ICACHE_RAM_ATTR
#else
#define ISR_PREFIX
#endif

LoRaClass::LoRaClass()
    : _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
      _spi(&LORA_DEFAULT_SPI), _ss(LORA_DEFAULT_SS_PIN),
      _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
      _frequency(0), _packetIndex(0), _implicitHeaderMode(0), _onReceive(NULL),
      _onTxDone(NULL) {
  // overide Stream timeout value
  setTimeout(0);
}

int LoRaClass::begin(long frequency) {
#if defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
  pinMode(LORA_IRQ_DUMB, OUTPUT);
  digitalWrite(LORA_IRQ_DUMB, LOW);

  // Hardware reset
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, LOW);

  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);
  digitalWrite(LORA_RESET, LOW);
  delay(200);
  digitalWrite(LORA_RESET, HIGH);
  delay(50);
#endif

  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);

  if (_reset != -1) {
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
  // Per errata Revision 1 - Sept 2013
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  sleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17);

  // put in standby mode
  idle();

  return 1;
}

/// Shut down the radio
void LoRaClass::end() {
  // put in sleep mode
  sleep();

  // stop SPI
  _spi->end();
}

/// Start the sequence of sending a packet.
///
/// \param implicitHeader (optional) true enables implicit header mode,
/// `false` enables explicit header mode (default) \return 1 if radio is ready
/// to transmit, 0 if busy or on failure.
int LoRaClass::beginPacket(int implicitHeader) {
  if (isTransmitting()) {
    return 0;
  }

  // put in standby mode
  idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and payload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

/// End the sequence of sending a packet.
///
/// \code
/// LoRa.endPacket();
/// LoRa.endPacket(async);
/// \endcode
///
/// \param async (optional) true enables non-blocking mode, false waits
/// for transmission to be completed (default) \return 1 on success, 0 on
/// failure.
int LoRaClass::endPacket(bool async) {

  if ((async) && (_onTxDone))
    writeRegister(REG_DIO_MAPPING_1, DIO0_TX_DONE); // DIO0 => TXDONE

  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (!async) {
    // wait for TX done
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      yield();
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}

bool LoRaClass::isTransmitting() {
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

/// Check if a packet has been received.
///  \code
///  int packetSize = LoRa.parsePacket();
///  int packetSize = LoRa.parsePacket(size);
///  \endcode
///  \param  size (optional) if > 0 implicit header mode is enabled with the
///  expected a packet of size bytes, default mode is explicit header mode
///  \return  the packet size in bytes or 0 if no packet was received.
int LoRaClass::parsePacket(int size) {
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) &&
      (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (readRegister(REG_OP_MODE) !=
             (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

/// Get the average Received Signal Strength Indicator (RSSI) of the last
/// received packet
///
/// \code
/// int rssi = LoRa.packetRssi();
/// \endcode
/// \return averaged RSSI of the last received packet (dBm).
int LoRaClass::packetRssi() {
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD
                                                  ? RSSI_OFFSET_LF_PORT
                                                  : RSSI_OFFSET_HF_PORT));
}

/// Get the Signal to Noise Ratio (SNR)
/// \code
/// float snr = LoRa.packetSnr();
/// \endcode
/// \return the estimated SNR of the received packet in dB.
float LoRaClass::packetSnr() {
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

/// Frequency error of the received packet. The frequency error is the
/// frequency offset between the receiver centre frequency and that of an
/// incoming LoRa signal.
/// \code
/// long freqErr = LoRa.packetFrequencyError();
/// \endcode
/// \return the frequency error of the received packet in Hz.
long LoRaClass::packetFrequencyError() {
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & B111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & B1000) { // Sign bit is on
    freqError -= 524288;                          // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5.
                            // Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) *
                       (getSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

/// Get current Received Signal Strength Indicator (RSSI).
/// RSSI can be read at any time (during packet reception or not)
///
/// \code
/// int rssi = LoRa.rssi();
/// \endcode
/// \return the current RSSI of the radio (dBm).
int LoRaClass::rssi() {
  return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD
                                              ? RSSI_OFFSET_LF_PORT
                                              : RSSI_OFFSET_HF_PORT));
}

/// Write data to the packet. Each packet can contain up to 255 bytes.
///
/// \note Other Arduino Print API's can also be used to write data into the
/// packet
/// \code
/// LoRa.write(byte);
/// \endcode
///
/// \param byte single byte to write to packet
///
/// \return the number of bytes written.
size_t LoRaClass::write(uint8_t byte) { return write(&byte, sizeof(byte)); }

/// Write data to the packet. Each packet can contain up to 255 bytes.
///
/// \note Other Arduino Print API's can also be used to write data into the
/// packet
/// \code
/// LoRa.write(buffer, length);
/// \endcode
///
/// \param buffer data to write to packet
/// \param size size of data to write
///
/// \return the number of bytes written.
size_t LoRaClass::write(const uint8_t *buffer, size_t size) {
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

/// Get number of bytes available for reading
/// \code
/// int availableBytes = LoRa.available()
/// \endcode
///
/// \return number of bytes available for reading
int LoRaClass::available() {
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

/// Read the next byte from the packet.
///
/// \code
/// byte b = LoRa.read();
/// \endcode
/// \return the next byte in the packet or -1 if no bytes are available.
int LoRaClass::read() {
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

/// Peek at the next byte in the packet.
///
/// \code
/// byte b = LoRa.peek();
/// \endcode
///
/// \return the next byte in the packet or -1 if no bytes are available.
int LoRaClass::peek() {
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

/// Override of Print::flush() that does nothing.
void LoRaClass::flush() {}

#ifndef ARDUINO_SAMD_MKRWAN1300
/// Register a callback function for when a packet is received.
///  \code
///  void callback(int packetSize) {...}
///  LoRa.onReceive(callback);
///  \endcode
///  \param callback callback function with signature void(int packetSize) to
///  call when a packet is received.
void LoRaClass::onReceive(RxFunction callback) {
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise,
                    RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

/// Register a callback function for when a packet transmission finishes.
///
/// \code
/// void txDone() {...}
/// LoRa.onTxDone(txDone);
/// \endcode
///
/// \param callback function to call when a packet transmission finishes.
void LoRaClass::onTxDone(TxFunction callback) {
  _onTxDone = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise,
                    RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

/// Register a callback function for when Channel Activity is Detected.
///
/// \code
/// void channelActivity() {...}
/// LoRa.detectChannelActivity();
/// ...
/// LoRa.onTxDone(channelActivity);
/// \endcode
///
/// \param callback function to call when channel activity is detected.
void LoRaClass::onCadDone(CadFunction callback) {
  _onCadDone = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise,
                    RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

/// Register callback for Frequency hopping spread spectrum (FHSS) channel
/// change.
///
/// \param callback function to be called when the FHSS channel changes.
void LoRaClass::onFhssChange(FhssChangeFunction callback) {
  _onFhssChange = callback;
  if (callback) {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise,
                    RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

/// Begin channel activity detection (CAD)
///
/// \code
/// void channelActivity() {...}
/// LoRa.detectChannelActivity();
/// ...
/// LoRa.onTxDone(channelActivity);
/// \endcode
void LoRaClass::detectChannelActivity(void) {
  writeRegister(REG_DIO_MAPPING_1, DIO0_CAD_DONE); // DIO0 => CADDONE
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

/// Puts the radio in continuous receive mode.
/// \code
/// LoRa.receive();
/// LoRa.receive(int size);
/// \endcode
/// \param size (optional) if > 0 implicit header mode is enabled with the
/// expected a packet of size bytes, default mode is explicit header mode The
/// onReceive callback will be called when a packet is received.
void LoRaClass::receive(int size) {

  writeRegister(REG_DIO_MAPPING_1, DIO0_RX_DONE); // DIO0 => RXDONE

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
#endif

/// Put the radio in idle (standby) mode.
/// \code
/// LoRa.idle();
/// \endcode
void LoRaClass::idle() {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/// Put the radio in sleep mode.
/// \todo explain what sleep mode does...
/// \code
/// LoRa.sleep();
/// \endcode
void LoRaClass::sleep() {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/// Change the TX power of the radio.
///
///  \code
///  LoRa.setTxPower(txPower);
///  LoRa.setTxPower(txPower, outputPin);
///  \endcode
///  \param level TX power in dB, defaults to 17
///  \param outputPin (optional) PA output pin, supported values are
///  `PA_OUTPUT_RFO_PIN` and `PA_OUTPUT_PA_BOOST_PIN`, defaults to
///  `PA_OUTPUT_PA_BOOST_PIN`. Supported values are 2 to 20 for
///  PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN. Most modules
///  have the PA output pin connected to PA BOOST,
void LoRaClass::setTxPower(int level, int outputPin) {
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      // Default value PA_HF/LF or +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

/// Change the frequency of the radio.
///
/// \code
/// LoRa.setFrequency(frequency);
/// \endcode
/// \param frequency frequency in Hz. One of (433000000L, 868000000L,
/// 915000000L).
void LoRaClass::setFrequency(long frequency) {
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

  errataCheck();
}

/// Get the current frequency of the radio as set in constructor or
/// setFrequency()
///
/// \return Current radio frequency.
long LoRaClass::getFrequency() const {
  // readRegister return uint8_t
  // Need to work in a larger type
  uint64_t lsb = readRegister(REG_FRF_LSB);
  uint64_t mid = readRegister(REG_FRF_MID);
  uint64_t msb = readRegister(REG_FRF_MSB);

  uint64_t frf = msb << 16 | mid << 8 | lsb;

  const uint64_t kF = 32000000;
  frf = (frf * kF) >> 19;
  return frf;
}

int LoRaClass::getSpreadingFactor() {
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

/// Change the spreading factor of the radio.
///
/// \code
/// LoRa.setSpreadingFactor(spreadingFactor);
/// \endcode
/// \param sf spreading factor, defaults to `7`
/// Supported values are between `6` and `12`.
/// If a spreading factor of `6` is set, implicit header mode must be used to
/// transmit and receive packets.
void LoRaClass::setSpreadingFactor(int sf) {
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2,
                (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long LoRaClass::getSignalBandwidth() {
  byte bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
  case 0:
    return 7.8E3;
  case 1:
    return 10.4E3;
  case 2:
    return 15.6E3;
  case 3:
    return 20.8E3;
  case 4:
    return 31.25E3;
  case 5:
    return 41.7E3;
  case 6:
    return 62.5E3;
  case 7:
    return 125E3;
  case 8:
    return 250E3;
  case 9:
    return 500E3;
  }

  return -1;
}

/// Change the signal bandwidth of the radio.
/// \code
/// LoRa.setSignalBandwidth(signalBandwidth);
/// \endcode
/// \param sbw signal bandwidth in Hz, defaults to 125000.
///  Supported values
///  are 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000,
///  and 500000.
void LoRaClass::setSignalBandwidth(long sbw) {
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1,
                (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  errataCheck();
  setLdoFlag();
}

void LoRaClass::errataCheck() {

  // Per errata Revision 1 - Sept 2013
  //
  // Devices SX1276, SX1277, and SX1278.
  //
  // The following LoRa registers should be changed as described, for BW=500 kHz
  // For carrier frequencies ranging from 862 to 1020 MHz
  //  o Set LoRa register at address 0x36 to value 0x02 (by default 0x03)
  //  o Set LoRa register at address 0x3a to value 0x64 (by default 0x65)
  //
  // For carrier frequencies ranging from 410 to 525 MHz
  //  o Set LoRa register at address 0x36 to value 0x02 (by default 0x03)
  //  o Set LoRa register at address 0x3a to value 0x7F (by default 0x65)
  if (_frequency > 862E6 && _frequency < 1020E6) {
    if (getSignalBandwidth() == 500E3) {
      writeRegister(REG_HIGH_BW_OPTIMIZE_1, 0x02);
      writeRegister(REG_HIGH_BW_OPTIMIZE_2, 0x64);
    }
  } else if (_frequency > 410E6 && _frequency < 525E6) {
    writeRegister(REG_HIGH_BW_OPTIMIZE_1, 0x02);
    writeRegister(REG_HIGH_BW_OPTIMIZE_2, 0x7F);
  }
}

void LoRaClass::setLdoFlag() {
  // Section 4.1.1.5
  long symbolDuration =
      1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()));

  // Section 4.1.1.6
  boolean ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
  bitWrite(config3, 3, ldoOn);
  writeRegister(REG_MODEM_CONFIG_3, config3);
}

/// Change the coding rate of the radio.
///
/// \code
/// LoRa.setCodingRate4(codingRateDenominator);
/// \endcode
/// \param denominator denominator of the coding rate, defaults to 5
/// Supported values are between 5 and 8, these correspond to coding rates of
/// 4/5 and 4/8. The coding rate numerator is fixed at 4.
void LoRaClass::setCodingRate4(int denominator) {
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1,
                (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/// Change the preamble length of the radio.
///
/// \code
/// LoRa.setPreambleLength(preambleLength);
/// \endcode
/// \param length preamble length in symbols, defaults to 8
/// Supported values are between 6 and 65535.
void LoRaClass::setPreambleLength(long length) {
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/// Change the sync word of the radio.
///
/// \code
/// LoRa.setSyncWord(syncWord);
/// \endcode
/// \param sw byte value to use as the sync word, defaults to 0x12
void LoRaClass::setSyncWord(int sw) { writeRegister(REG_SYNC_WORD, sw); }

void LoRaClass::enableCrc() {
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc() {
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableInvertIQ() {
  writeRegister(REG_INVERTIQ, 0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ() {
  writeRegister(REG_INVERTIQ, 0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}

/// Enables overload current protection (OCP) for Power Amplifier (PA)
///
/// \todo create a disableOCP?
///
/// \param mA
/// OcpTrim   | \f$I_{max}\f$ | Formula
/// --------- | ------------- | ---------------------
/// 0 to 15   | 45 to 120 mA  | 45 + 5*OcpTrim [mA]
/// 16 to 27  | 130 to 240 mA | -30 + 10*OcpTrim [mA]
/// 27+       | 240 mA        | 240 mA
///
/// Default \f$I_{max}\f$ = 100mA
void LoRaClass::setOCP(uint8_t mA) {
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <= 240) {
    ocpTrim = (mA + 30) / 10;
  }

  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

/// Set Low Noise Amplifier (LNA) Gain for better RX sensitivity, by default AGC
/// (Automatic Gain Control) is used and LNA gain is not used.
///
/// \code
/// LoRa.setGain(gain);
/// \endcode
/// \param gain LNA gain
/// Supported values are between 0 and 6. If gain is 0, AGC will be enabled and
/// LNA gain will not be used. Else if gain is from 1 to 6, AGC will be disabled
/// and LNA gain will be used.
void LoRaClass::setGain(uint8_t gain) {
  // check allowed range
  if (gain > 6) {
    gain = 6;
  }

  // set to standby
  idle();

  // set gain
  if (gain == 0) {
    // if gain = 0, enable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
  } else {
    // disable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x00);

    // clear Gain and set LNA boost
    writeRegister(REG_LNA, 0x03);

    // set gain
    writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
  }
}

/// Generate a random byte, based on the Wideband RSSI measurement.
///
/// \code
/// byte b = LoRa.random();
/// \endcodde
/// \return random byte.
byte LoRaClass::random() { return readRegister(REG_RSSI_WIDEBAND); }

/// Override the default `NSS`, `NRESET`, and `DIO0` pins used by the library.
/// **Must** be called before `LoRa.begin()`.
/// \code
/// LoRa.setPins(ss, reset, dio0);
/// \endcode
/// \param ss new slave select pin to use, defaults to 10
/// \param reset new reset pin to use, defaults to 9
/// \param dio0 new DIO0 pin to use, defaults to 2.  **Must** be interrupt
/// capable via
/// [attachInterrupt(...)](https://www.arduino.cc/en/Reference/AttachInterrupt).
/// This call is optional and only needs to be used if you need to change the
/// default pins used.
void LoRaClass::setPins(int ss, int reset, int dio0) {
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

/// Override the default SPI interface used by the library. **Must** be called
/// before `LoRa.begin()`.
///
/// \code
/// LoRa.setSPI(spi);
/// \endcode
/// \param spi  new SPI interface to use, defaults to SPI
/// This call is optional and only needs to be used if you need to change the
/// default SPI interface used, in the case your Arduino (or compatible) board
/// has more than one SPI interface present.
void LoRaClass::setSPI(SPIClass &spi) { _spi = &spi; }

/// Override the default SPI frequency of 10 MHz used by the library. **Must**
/// be called before LoRa.begin(). \code LoRa.setSPIFrequency(frequency);
/// \endcode
/// \param frequency new SPI frequency to use, defaults to 8000000
/// This call is optional and only needs to be used if you need to change the
/// default SPI frequency /// used. Some logic level converters cannot support
/// high speeds such as 8 MHz, so a lower SPI frequency can be selected with
/// `LoRa.setSPIFrequency(frequency)`.
void LoRaClass::setSPIFrequency(uint32_t frequency) {
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

/// Output the values of the registers in the LoRa hardware
/// This is useful for debugging purposes.
/// \param out Stream to send output to
void LoRaClass::dumpRegisters(Stream &out) {
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void LoRaClass::explicitHeaderMode() {
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode() {
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::handleDio0Rise() {
  // This is called from an interrupt routine (onDioRise)
  // It must therefore be as fast as possible.
  //
  // Called for any of:
  // IRQ_RX_TIMEOUT_MASK
  // IRQ_RX_DONE_MASK
  // IRQ_PAYLOAD_CRC_ERROR_MASK
  // IRQ_VALID_HEADER_MASK
  // IRQ_TX_DONE_MASK
  // IRQ_CAD_DONE_MASK
  // IRQ_FHSS_CHANGE_CH_MASK
  // IRQ_CAD_DETECTED_MASK
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  // Writing a 1 clears the flag in the hardware.
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_FHSS_CHANGE_CH_MASK) != 0) {
    if (_onFhssChange) {
      _onFhssChange();
    }
  }
  if ((irqFlags & IRQ_CAD_DONE_MASK) != 0) {
    if (_onCadDone) {
      _onCadDone((irqFlags & IRQ_CAD_DETECTED_MASK) != 0);
    }
  } else if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

    if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
      // received a packet
      _packetIndex = 0;

      // read packet length
      int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH)
                                             : readRegister(REG_RX_NB_BYTES);

      // set FIFO address to current RX address
      writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

      if (_onReceive) {
        _onReceive(packetLength);
      }
    } else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
      if (_onTxDone) {
        _onTxDone();
      }
    }
  }
}

uint8_t LoRaClass::readRegister(uint8_t address) const {
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value) {
  singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value) const {
  uint8_t response;

  digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  _spi->transfer(address);
  response = _spi->transfer(value);
  _spi->endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}

ISR_PREFIX void LoRaClass::onDio0Rise() { LoRa.handleDio0Rise(); }

LoRaClass LoRa;
