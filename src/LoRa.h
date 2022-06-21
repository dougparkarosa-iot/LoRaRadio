// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>

#define USE_LORA_FUNCTIONAL_CALLBACK 0
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
#define LORA_DEFAULT_SPI SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define LORA_DEFAULT_SS_PIN 10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN 2
#endif

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

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
  void onTxDone(TxFunction callback);

  void receive(int size = 0);
#endif

  void setMode(DeviceMode mode);
  DeviceMode getMode();
  void singleReceive();

  void idle();

  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);

  void setFrequency(long frequency);

  long getFrequency();

  void setSpreadingFactor(int sf);

  void setSignalBandwidth(long sbw);

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
  /// @}

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

  void detectChannelActivity(void);

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

  bool isTransmitting();

private:
  void errataCheck();
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  int getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  uint8_t readRegister(uint8_t address);
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
  RxFunction _onReceive;
  TxFunction _onTxDone;
};

extern LoRaClass LoRa;

#endif
