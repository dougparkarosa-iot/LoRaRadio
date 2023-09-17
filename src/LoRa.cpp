// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#include <LoRa.h>

#define USE_DEBUG_OUTPUT 0
#define USE_ERRATTA_CHECK 0

#if USE_DEBUG_OUTPUT
#include <Ansi.h>
#include <AnsiTermLogger.h>
#include <PreserveStreamFlags.h>
#include <TerminalOut.h>
#include <iomanip>
#include <iostream>
#endif // USE_DEBUG_OUTPUT

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
#define REG_HOP_CHANNEL 0x1c
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_SYMB_TIMEOUT_MSG_MASK 0x3
#define REG_SYMB_TIMEOUT_LSB 0x1f // MSB in REG_MODEM_CONFIG_2 bits 1-0
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
#define REG_IF_FREQ_1 0x30
#define REG_IF_FREQ_2 0x2F

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
#define MODE_MASK 0x07
#define MODE_CAD 0x07

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_RX_TIMEOUT_MASK 0x80
#define IRQ_RX_DONE_MASK 0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_VALID_HEADER_MASK 0x10
#define IRQ_CAD_DONE_MASK 0x04
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_FhssChangeChannel_Mask 0x02
#define IRQ_CAD_DETECTED_MASK 0x01

#define CRC_ON_MASK 0x04
#define CRC_ON_PAYLOAD_MASK 0x40
#define PLL_TIMEOUT 0x80

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
      _frequency(0), _packetIndex(0), _implicitHeaderMode(0), _crcErrorCount(0),
      _requireCRC(true), _onReceive(NULL), _onCadDone(NULL), _onTxDone(NULL) {
  // override Stream timeout value
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
  setTxPower(17); // Tried 17

  // put in standby mode
  idle();

  return 1;
}

/// Shut down the radio
void LoRaClass::end() {
  // put in sleep mode
  sleep();

  // unhook interrupts.
  _onTxDone = nullptr;
  _onReceive = nullptr;

  detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
  SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif

  // stop SPI
  _spi->end();
}

/// Set the receive timeout.
/// From the SX1276/77/78/79 datasheet:
/// > In this mode, the modem searches for a preamble during a given period of
/// > time. If a preamble hasn’t been found at the end of the time window, the
/// > chip generates the RxTimeout interrupt and goes back to Standby mode. The
/// > length of the reception window (in symbols) is defined by the
/// > RegSymbTimeout register and should be in the range of 4 (minimum time for
/// > the modem to acquire lock on a preamble) up to 1023 symbols.
///
/// \param timeout value in the range 4 to 1023 symbols. Default ix 0x64 (100).
void LoRaClass::setRxTimeout(int timeout) {
  if (timeout < 4) {
    timeout = 4;
  }
  if (timeout > 1023) {
    timeout = 1023;
  }
  uint8_t msb = REG_SYMB_TIMEOUT_MSG_MASK & (timeout >> 8);
  uint8_t lsb = timeout & 0xff;

  // Set msb into modem config 2 bits 0-1 retaining the higher order
  // bits already there.
  auto mConfig2 = readRegister(REG_MODEM_CONFIG_2) & ~REG_SYMB_TIMEOUT_MSG_MASK;
  writeRegister(REG_MODEM_CONFIG_2, mConfig2 | (msb));

  // Set lsb bits. Most often this should be all that changes.
  writeRegister(REG_SYMB_TIMEOUT_LSB, lsb);
}

/// Get the current RxTimeout
/// \return receive timeout in symbols in the range 4-1023.
int LoRaClass::getRxTimeout() {
  auto msb = readRegister(REG_MODEM_CONFIG_2) & REG_SYMB_TIMEOUT_MSG_MASK;
  auto lsb = readRegister(REG_SYMB_TIMEOUT_LSB);
  return (msb << 8) | lsb;
}

/// Start the sequence of sending a packet.
///
/// \param implicitHeader (optional) true enables implicit header mode,
/// `false` enables explicit header mode (default)
/// \return 1 if radio is ready
/// to transmit, 0 if busy or on failure.
int LoRaClass::beginPacket(int implicitHeader) {
  if (isTransmitting()) {
    return 0;
  }

  // put in standby mode
  // Must be in standby mode to write to FIFO buffer.
  ensureConfigWritable();

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
/// for transmission to be completed (default)
/// \return 1 on success, 0 on
/// failure.
int LoRaClass::endPacket(bool async) {

  if ((async) && (_onTxDone)) {
    // Turn on TXDONE interrupt
    writeRegister(REG_DIO_MAPPING_1, DIO0_TX_DONE); // DIO0 => TXDONE
  }

  if (_requireCRC) {
    enableCrc();
  } else {
    disableCrc();
  }

  // put in TX mode
  // writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  setMode(DeviceMode::TX);

  if (!async) {
    // wait for TX done
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      yield(); // equivalent to vPortYield() on ESP32.
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}

/// Set current device mode
/// \param mode Mode to select
void LoRaClass::setMode(DeviceMode mode) {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
  // Mode change takes a few micro seconds.
  delay(1);
}

/// Get current device mode
/// \return the current device mode.
LoRaClass::DeviceMode LoRaClass::getMode() {
  return static_cast<DeviceMode>(readRegister(REG_OP_MODE) & MODE_MASK);
}

/// Check to see if radio is busy transmitting.
/// \return true if transmitting false otherwise.
bool LoRaClass::isTransmitting() {
  // When in TX mode transmission continues until done then
  // automatically drops to stand by mode after firing off
  // the TX Done interrupt.
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  // If we got here then we are not in transmit mode.
  // handleDio0Rise should clear this if it is hooked.
  // endPacket() clears this in the non async mode, so this
  // seems like it isn't necessary.
  //
  // It is safe to clear this now.
  // Clear the TX Done interrupt request flag.
  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

/// Check if a packet has been received.
/// If the LoRa radio is not already in single receive mode and there
/// is not data to read single receive mode is entered. See singleReceive().s
///  \code
///  int packetSize = LoRa.parsePacket();
///  int packetSize = LoRa.parsePacket(size);
///  \endcode
///  \param  size (optional) if > 0 implicit header mode is enabled with the
///  expected a packet of size bytes, default mode is explicit header mode
///  \return  the packet size in bytes or 0 if no packet was received.
int LoRaClass::parsePacket(int size) {
  int packetLength = 0;

  // Pick up IRQ flags for later use.
  int irqFlags = readRegister(REG_IRQ_FLAGS);
#if USE_DEBUG_OUTPUT
  if (irqFlags != 0 && irqFlags != 0x80) {
    PreserveStreamFlags p(std::cout);
    SBNPL::AnsiOut(kDebugLine)
        << "irqFlags " << std::setw(4) << std::hex << std::showbase << irqFlags
        << " SR: " << ((getMode() == DeviceMode::RXSINGLE) ? "T" : "F")
        << " CRC: " << std::dec << _crcErrorCount << std::flush;
  }
#endif // USE_DEBUG_OUTPUT

  // From datasheet
  // In order to retrieve received data from FIFO the user must ensure that
  // ValidHeader, PayloadCrcError, RxDone and RxTimeout interrupts in the status
  // register RegIrqFlags are not asserted to ensure that packet reception has
  // terminated successfully (i.e. no flags should be set).
  //
  // Note: RxDone should be set. The above from the datasheet is imprecise.
  //
  // In case of errors the steps below should be skipped and the packet
  // discarded. In order to retrieve valid received data from
  // the FIFO the user must:
  //   * RegRxNbBytes Indicates the number of bytes that have been received thus
  //     far.
  //   * RegFifoAddrPtr is a dynamic pointer that indicates precisely where
  //     the Lora modem received data has been written up to.
  //   * Set RegFifoAddrPtr to RegFifoRxCurrentAddr. This sets the FIFO pointer
  //     to the location of the last packet received in the FIFO. The payload
  //     can then be extracted by reading the register RegFifo, RegRxNbBytes
  //     times.
  //   * Alternatively, it is possible to manually point to the location of the
  //     last packet received, from the start of the current packet, by setting
  //     RegFifoAddrPtr to RegFifoRxByteAddr minus RegRxNbBytes. The payload
  //     bytes can then be read from the FIFO by reading the RegFifo address
  //     RegRxNbBytes times.

  // Pick up the things we want to know
  const bool rx_done = (irqFlags & IRQ_RX_DONE_MASK) != 0;
  const bool valid_header = (irqFlags & IRQ_VALID_HEADER_MASK) != 0;
  const bool crc_error = (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0;
  const bool timeout = (irqFlags & IRQ_RX_TIMEOUT_MASK) != 0;

  // By observation this routine may be called several times after the
  // VALID_HEADER is set. It is probably wrong to clear this before
  // RX_DONE is set.

  bool rejectAsCRCError = false;
  if (rx_done) {
    if (crc_error) {
      _crcErrorCount++;
      rejectAsCRCError = true;
    } else {
      if (_requireCRC && !isCRCOnPayload()) {
        // Reject. Maybe phantom packet or not one we want.
        // Count this as a crc error.
        _crcErrorCount++;
        rejectAsCRCError = true;
      }
    }
#if USE_DEBUG_OUTPUT
    SBNPL::AnsiOut(kDebugLine + 3)
        << "CRC? " << (isCRCOnPayload() ? "Yes" : "No") << " Reject? "
        << (rejectAsCRCError ? "Yes" : "No") << std::endl;
#endif // USE_DEBUG_OUTPUT
  }

  if (rx_done && !rejectAsCRCError && !timeout && valid_header) {

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);
#if USE_DEBUG_OUTPUT
    // received a packet
    SBNPL::AnsiOut(kDebugLine + 1) << "Valid Packet Received" << std::endl;
#endif // USE_DEBUG_OUTPUT
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address, see above notes
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    // Should already be in standby mode.
    idle();
  } else if (getMode() != DeviceMode::RXSINGLE) {

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    // not currently in RX mode
#if USE_DEBUG_OUTPUT
    // SBNPL::AnsiOut(kDebugLine + 2) << "singleReceive" << std::endl;
#endif // USE_DEBUG_OUTPUT

    // Seems better to do this only when we are switching into single receive
    if (size > 0) {
      if (_requireCRC) {
        enableCrc();
      } else {
        disableCrc();
      }

      implicitHeaderMode();

      writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
      disableCrc(); // Uses header to determine if CRC is to be checked.
      explicitHeaderMode();
    }

    singleReceive();
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
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
    freqError -= 524288;                           // 0b1000'0000'0000'0000'0000
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

/// Register an interrupt callback function for when a packet transmit is done.
///  \param callback callback function with signature void(int packetSize) to
///  call when a packet is received.
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

/// Switch to continuous receive mode.
///
/// \param size Packet size for implicitHeader mode.
/// will put the radio into implicit header mode when
/// size is not zero. Default is zero which is for explicit header mode.
void LoRaClass::receive(int size) {

  writeRegister(REG_DIO_MAPPING_1, DIO0_RX_DONE); // DIO0 => RXDONE

  if (size > 0) {
    if (_requireCRC) {
      enableCrc();
    } else {
      disableCrc();
    }

    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    disableCrc(); // Uses header to tell if it is on or off.
    explicitHeaderMode();
  }

  // writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
  setMode(DeviceMode::RXCONTINUOUS);
}

void LoRaClass::channelActivityDetection(void) {
  writeRegister(REG_DIO_MAPPING_1, DIO0_CAD_DONE); // DIO0 => CADDONE
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}
#endif

/// Put the LoRa radio into single receive mode.
///
/// \code
///  LoRa.singleReceive();
///  int packetSize = LoRa.parsePacket();
///  if (packetSize) {
///    // received a packet
///    Serial.print("Received packet '");
///
///    // read packet
///    while (LoRa.available()) {
///      Serial.print((char)LoRa.read());
///    }
///
///    // print RSSI of packet
///    Serial.print("' with RSSI ");
///    Serial.println(LoRa.packetRssi());
///  }
/// \endcode
void LoRaClass::singleReceive() {
  ensureConfigWritable();

  // reset FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, 0);

  // Clear IRQ's
  // writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_MASK);

  // put in single RX mode
  setMode(DeviceMode::RXSINGLE);
  // writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  //  Mode changes take a few micro seconds.
  // delay(1);
}

/// Put the radio in idle (standby) mode.
/// \code
/// LoRa.idle();
/// \endcode
void LoRaClass::idle() { setMode(DeviceMode::STDBY); }

/// Put the radio in sleep mode.
/// \todo explain what sleep mode does...
/// \code
/// LoRa.sleep();
/// \endcode
void LoRaClass::sleep() { setMode(DeviceMode::SLEEP); }

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
long LoRaClass::getFrequency() {
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

/// Get the current spreading factor
/// \return the spreading factor.
int LoRaClass::getSpreadingFactor() {
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

/// Change the spreading factor of the radio.
/// \code
/// LoRa.setSpreadingFactor(spreadingFactor);
/// \endcode
/// \param sf spreading factor, defaults to 7
/// Supported values are between 6 and 12. If a spreading factor of 6 is set,
/// implicit header mode must be used to transmit and receive packets.
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
    return 7800;
  case 1:
    return 10400;
  case 2:
    return 15600;
  case 3:
    return 20800;
  case 4:
    return 31250;
  case 5:
    return 41700;
  case 6:
    return 62500;
  case 7:
    return 125000;
  case 8:
    return 250000;
  case 9:
    return 500000;
  }

  return -1;
}

/// Change the signal bandwidth of the radio.
/// \code
/// LoRa.setSignalBandwidth(signalBandwidth);
/// \endcode
/// \param sbw signal bandwidth in Hz, defaults to 125000.
/// Supported values are 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000,
/// 250000, and 500000.
void LoRaClass::setSignalBandwidth(long sbw) {
  int bw;

  if (sbw <= 7800) {
    bw = 0;
  } else if (sbw <= 10400) {
    bw = 1;
  } else if (sbw <= 15600) {
    bw = 2;
  } else if (sbw <= 20800) {
    bw = 3;
  } else if (sbw <= 31250) {
    bw = 4;
  } else if (sbw <= 41700) {
    bw = 5;
  } else if (sbw <= 62500) {
    bw = 6;
  } else if (sbw <= 125000) {
    bw = 7;
  } else if (sbw <= 250000) {
    bw = 8;
  } else /*if (sbw <= 500000)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1,
                (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  errataCheck();
  setLdoFlag();
}

void LoRaClass::errataCheck() {
#if USE_ERRATTA_CHECK
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
    if (getSignalBandwidth() == 500000) {
      writeRegister(REG_HIGH_BW_OPTIMIZE_1, 0x02);
      writeRegister(REG_HIGH_BW_OPTIMIZE_2, 0x64);
    }
  } else if (_frequency > 410E6 && _frequency < 525E6) {
    writeRegister(REG_HIGH_BW_OPTIMIZE_1, 0x02);
    writeRegister(REG_HIGH_BW_OPTIMIZE_2, 0x7F);
  }

  // Errata 2.3 - receiver spurious reception of a LoRa signal
  // This is incomplete based on errata. It doesn't hanlde bandwidths less than
  // 62500.

  // Should be in sleep or standby for this to work
  auto bw = getSignalBandwidth();
  auto rDetectOptimize = (readRegister(REG_DETECTION_OPTIMIZE) & 0x78) | 0x03;
  if (bw < 500000) {
    writeRegister(REG_DETECTION_OPTIMIZE, rDetectOptimize);
    writeRegister(REG_IF_FREQ_2, 0x40);
    writeRegister(REG_IF_FREQ_1, 0x00);
    // writeRegister(REG_IF_FREQ_1, 0x40);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, rDetectOptimize | 0x80);
  }
#endif // USE_ERRATTA_CHECK
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
  writeRegister(REG_MODEM_CONFIG_2,
                readRegister(REG_MODEM_CONFIG_2) | CRC_ON_MASK);
}

void LoRaClass::disableCrc() {
  writeRegister(REG_MODEM_CONFIG_2,
                readRegister(REG_MODEM_CONFIG_2) & ~CRC_ON_MASK);
}

bool LoRaClass::isCRCOnPayload() {
  return readRegister(REG_HOP_CHANNEL | CRC_ON_PAYLOAD_MASK) != 0;
}

/// Check to see if PLL timed out on TX, RX, or CAD
/// \return true if timeout occured.
bool LoRaClass::isPLLTimeout() {
  return readRegister(REG_HOP_CHANNEL | PLL_TIMEOUT) != 0;
}

bool LoRaClass::isCRCRequired() { return _requireCRC; }

void LoRaClass::setRequireCRC(bool requireCrc) { _requireCRC = requireCrc; }

void LoRaClass::enableInvertIQ() {
  // This seems wrong. Spec say's bit 6 for RX, 0 for TX, 1-5 is 0x13
  // So this would be 0x40 | 0x13 -> 0x53
  // The following discusses this problem with another library
  // https://blog.devmobile.co.nz/tag/reginvertiq/
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
/// \code
/// LoRa.setGain(gain);
/// \endcode
/// \param gain LNA gain. Supported values are
/// between 0 and 6. If gain is 0, AGC will be enabled and LNA gain will not be
/// used.Else if gain is from 1 to 6, AGC will be disabled and LNA gain will be
/// used.
void LoRaClass::setGain(uint8_t gain) {
  ensureConfigWritable();
  // check allowed range
  if (gain > 6) {
    gain = 6;
  }

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
/// \endcode
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

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 1) {
    _crcErrorCount++;
  }

  // clear IRQ's
  // Writing a 1 clears the flag in the hardware.
  writeRegister(REG_IRQ_FLAGS, irqFlags);

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

/// Read the value of a particular LoRa radio register.
/// Useful for testing.
/// \param address register to read.
/// \return value of the register specified by address.
uint8_t LoRaClass::readRegister(uint8_t address) {
  return singleTransfer(address & 0x7f, 0x00);
}

bool LoRaClass::isSleepingOrStandby() {
  auto mode = getMode();
  return (mode == DeviceMode::SLEEP) || (mode == DeviceMode::STDBY);
}

// 4.1.2.1 Configuration registers should be written only in Sleep and Standby
// Modes. Datasheet is not clear about which registers are configuration
// registers.
void LoRaClass::ensureConfigWritable() {
  if (!isSleepingOrStandby()) {
    setMode(DeviceMode::STDBY);
  }

  return;
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value) {
  singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value) {
  uint8_t response;

  _spi->beginTransaction(_spiSettings);
  digitalWrite(_ss, LOW);
  _spi->transfer(address);
  response = _spi->transfer(value);
  digitalWrite(_ss, HIGH);
  _spi->endTransaction();

  return response;
}

ISR_PREFIX void LoRaClass::onDio0Rise() { LoRa.handleDio0Rise(); }

LoRaClass LoRa;
