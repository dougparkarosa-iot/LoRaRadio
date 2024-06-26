// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#ifndef LORA_H
#define LORA_H

/// \file
/// \brief LoRa radio driver.

#include <Arduino.h>
#include <SPI.h>

#define USE_EXPERIMENTAL_CAD 1 ///< Enable experimental CAD support

#define USE_LORA_FUNCTIONAL_CALLBACK 1 ///< Enable functional callback support
#if USE_LORA_FUNCTIONAL_CALLBACK
#include <functional>
#endif

#if defined(ARDUINO_SAMD_MKRWAN1300)
#define LORA_DEFAULT_SPI SPI1
#define LORA_DEFAULT_SPI_FREQUENCY 200000
#define LORA_DEFAULT_SS_PIN LORA_IRQ_DUMB
#define LORA_DEFAULT_RESET_PIN -1
#define LORA_DEFAULT_DIO0_PIN -1
#elif defined(ARDUINO_SAMD_MKRWAN1310)
#define LORA_DEFAULT_SPI SPI1
#define LORA_DEFAULT_SPI_FREQUENCY 200000
#define LORA_DEFAULT_SS_PIN LORA_IRQ_DUMB
#define LORA_DEFAULT_RESET_PIN -1
#define LORA_DEFAULT_DIO0_PIN LORA_IRQ
#else
#define LORA_DEFAULT_SPI SPI           ///< Default SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6 ///< Default SPI frequency
#define LORA_DEFAULT_SS_PIN 10         ///< Default SS pin
#define LORA_DEFAULT_RESET_PIN 9       ///< Default reset pin
#define LORA_DEFAULT_DIO0_PIN 2        ///< Default DIO0 pin
#endif

#define PA_OUTPUT_RFO_PIN 0      ///< RFO output pin
#define PA_OUTPUT_PA_BOOST_PIN 1 ///< PA Boost output pin

/// \brief Cloned LoRaClass from Sandeep Mistry
///
/// Clone is here to allow changes as needed.
///
/// Callback structure uses std::function.
class LoRaClass : public Stream {
public:
#if USE_LORA_FUNCTIONAL_CALLBACK
  using RxFunction = std::function<void(int)>; ///< Callback type for receive
                                               ///< argument is packetSize
  using TxFunction = std::function<void()>;    ///< Callback type for transmit
  using CadFunction = std::function<void(
      boolean)>; ///< Callback type for Channel Activity Detect (CAD)
  using FhssChangeFunction =
      std::function<void()>; ///< Callback type for FhssChange
#else
  using RxFunction = void (*)(int); ///< Callback type for receive
  using TxFunction = void (*)();    ///< Callback type for transmit
  using CadFunction =
      void (*)(boolean); ///< Callback type for Channel Activity Detect (CAD)
  using FhssChangeFunction = void (*)(); ///< Callback type for FhssChange
#endif

  /// Device modes
  enum DeviceMode {
    SLEEP,        ///< Sleep (low power) mode
    STDBY,        ///< Standby mode
    FSTX,         ///< Frequency synthesis TX
    TX,           ///< Transmit
    FSRX,         ///< Frequency synthesis RX
    RXCONTINUOUS, ///< Receive continuous
    RXSINGLE,     ///< Receive single
    CAD           ///< Channel activity detection
  };

  /// Constructor
  LoRaClass();

  /// Connect to LoRa hardware using the input frequency
  ///
  /// \param frequency Frequency One of (433000000L, 868000000L, 915000000L).
  ///
  /// \return 1 if successful 0 otherwise
  int begin(long frequency);

  void end();

  int beginPacket(int implicitHeader = false);

  int endPacket(bool async = false);

  void setRxTimeout(int timeout = 0x64);
  int getRxTimeout();

  int parsePacket(int size = 0);
  int packetRssi();

  float packetSnr();

  long packetFrequencyError();

  int rssi();

  // from Print

  virtual size_t write(uint8_t byte);

  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream

  int available() override;

  int read() override;

  int peek() override;

  void flush() override;

#ifndef ARDUINO_SAMD_MKRWAN1300
  void onReceive(RxFunction callback);
  void onCadDone(CadFunction callback);
  void onTxDone(TxFunction callback);

  void receive(int size = 0);
  void channelActivityDetection(void);
#endif

  void setMode(DeviceMode mode);
  DeviceMode getMode();

  void idle();

  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);

  void setFrequency(long frequency);

  long getFrequency();

  void setSpreadingFactor(int sf);
  int getSpreadingFactor();

  void setSignalBandwidth(long signalBandwidth);
  long getSignalBandwidth();

  void setCodingRate4(int denominator);

  void setPreambleLength(long length);

  void setSyncWord(int sw);

  /// @{
  /// Enable or disable CRC usage, by default a CRC is not used.
  /// \code
  /// LoRa.enableCrc();
  ///
  /// LoRa.disableCrc();
  /// \endcode
  void enableCrc();
  void disableCrc();
  bool isCRCOnPayload();
  bool isCRCRequired();
  void setRequireCRC(bool requireCrc = true);
  /// @}

  bool isPLLTimeout();

#if USE_EXPERIMENTAL_CAD
  /// Check for channel activity.
  /// Will wait until CAD is finished and return.
  /// \return true if there is channel activity.
  bool isChannelActive();
#endif // USE_EXPERIMENTAL_CAD

  /// @{
  /// Enable or disable Invert the LoRa “in-phase” and “quadrature” (I and Q)
  /// signals, by default a invertIQ is not used.
  ///
  /// \code
  /// LoRa.enableInvertIQ();
  ///
  /// LoRa.disableInvertIQ();
  /// \endcode
  void enableInvertIQ();
  void disableInvertIQ();
  /// @}

  void setOCP(uint8_t mA); // Over Current Protection control

  void setGain(uint8_t gain); // Set LNA gain

  /// Generate a random byte, based on the Wideband RSSI measurement.
  ///
  /// \code
  /// byte b = LoRa.random();
  /// \endcode
  /// \return random byte.
  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN,
               int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPI(SPIClass &spi);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream &out);
  uint8_t readRegister(uint8_t address);

  bool isTransmitting();
  bool isSleepingOrStandby();
  bool isReceiving();
  bool isPacketReady();
  bool isReceiveTimeout();

  /**
   * @brief Resets the CRC error count.
   *
   * This function resets the count of CRC errors to zero.
   */
  void resetCRCErrorCount() { _crcErrorCount = 0; }
  /**
   * @brief Get the CRC error count.
   *
   * This function returns the number of CRC errors that have occurred.
   *
   * @return The number of CRC errors.
   */
  int getCRCErrorCount() const { return _crcErrorCount; }

  void singleReceive();

private:
  void errataCheck();
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  void setLdoFlag();

  void ensureConfigWritable();

  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();

private:
  SPISettings _spiSettings;
  SPIClass *_spi;
  int _ss;
  int _reset;
  int _dio0;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  int _crcErrorCount;
  bool _requireCRC;
  RxFunction _onReceive;
  CadFunction _onCadDone;
  TxFunction _onTxDone;
};

extern LoRaClass LoRa; ///< LoRa radio driver instance

#endif
