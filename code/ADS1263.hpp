/*
 * ADS1263.hpp
 *
 *  Created on: 23.08.2017
 *      Author: rico
 */

#ifndef ADS1263_HPP_
#define ADS1263_HPP_

//#include "shared.hpp"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"

namespace ADS1263 {

typedef struct
{
    uint32_t port;
    uint32_t number;
    uint32_t periph;
} Pin;

typedef enum {
    kADC1,
    kADC2,
} ADCNumber;

namespace IDAC {

typedef enum {
    kIDACModule1,
    kIDACModule2,
} Module;

typedef enum {
    k00_00_uA = 0x00,    ///default
    k50_uA = 0x01,
    k100_uA = 0x02,
    k250_uA = 0x03,
    k500_uA = 0x04,
    k750_uA = 0x05,
    k1000_uA = 0x06,
    k1500_uA = 0x07,
    k2000_uA = 0x08,
    k2500_uA = 0x09,
    k3000_uA = 0x0A,
} Magnitude;

typedef enum {
    kAIN0 = 0x00,
    kAIN1 = 0x01,
    kAIN2 = 0x02,
    kAIN3 = 0x03,
    kAIN4 = 0x04,
    kAIN5 = 0x05,
    kAIN6 = 0x06,
    kAIN7 = 0x07,
    kAIN8 = 0x08,
    kAIN9 = 0x09,
    kAINCOM = 0x0A,
    kNoConnection = 0x0B,  ///default
} Channel;

typedef struct {
    Channel channel;
    Magnitude magnitude;
    Module module;
} Setup;

}  // namespace IDAC

namespace SSI {

typedef enum {
    kSSI0_BASE = SSI0_BASE,
    kSSI1_BASE = SSI1_BASE,
    kSSI2_BASE = SSI2_BASE,
    kSSI3_BASE = SSI3_BASE,
} SsiBase;

}  // namespace SSI

typedef enum {
    kAIN0 = 0x00,  ///default positive
    kAIN1 = 0x01,  ///default negative
    kAIN2 = 0x02,
    kAIN3 = 0x03,
    kAIN4 = 0x04,
    kAIN5 = 0x05,
    kAIN6 = 0x06,
    kAIN7 = 0x07,
    kAIN8 = 0x08,
    kAIN9 = 0x09,
    kAINCOM = 0x0A,
    KTemperatureSensor = 0x0B,
    kAnalogPowerMonitor = 0x0C,
    kDigitalPowerMonitor = 0x0D,
    kDACTestSignalPositiv = 0x0E,
    kNoConnection = 0x0F,
} ADCChannel;

typedef enum {
    kSinc1 = 0x00,
    kSinc2 = 0x01,
    kSinc3 = 0x02,
    kSinc4 = 0x03,
    kFIR = 0x04,  /// (default)
} Filter;

typedef enum {
    k2_5_SPS = 0x00,
    k5_SPS = 0x01,
    k10_SPS = 0x02,
    k16_6_SPS = 0x03,
    k20_SPS = 0x04,  /// (default)
    k50_SPS = 0x05,
    k60_SPS = 0x06,
    k100_SPS = 0x07,
    k400_SPS = 0x08,
    k1200_SPS = 0x09,
    k2400_SPS = 0x0A,
    k4800_SPS = 0x0B,
    k7200_SPS = 0x0C,
    k14400_SPS = 0x0D,
    k19200_SPS = 0x0E,
    k38400_SPS = 0x0F,
} ADC1DataRate;

typedef enum {
    k10_SPS_2 = 0x00,  ///(default)
    k100_SPS_2 = 0x01,
    k400_SPS_2 = 0x02,
    k800_SPS_2 = 0x03,
} ADC2DataRate;

typedef enum {
    kId = 0x00,
    kPower = 0x01,
    kInterface = 0x02,
    kMode0 = 0x03,
    kMode1 = 0x04,
    kMode2 = 0x05,
    kInputMux = 0x06,
    kOffsetCalibration0 = 0x07,
    kOffsetCalibration1 = 0x08,
    kOffsetCalibration2 = 0x09,
    kFullScaleCalibration0 = 0x0A,
    kFullScaleCalibration1 = 0x0B,
    kFullScaleCalibration2 = 0x0C,
    kIdacMux = 0x0D,
    kIdacMagnitude = 0x0E,
    kReferenceMux = 0x0F,
    kTestDacP = 0x10,
    kTestDacN = 0x11,
    kGpioConnection = 0x12,
    kGpioDirection = 0x13,
    kGpioData = 0x14,
    kAdc2Configuration = 0x15,
    kAdc2Mux = 0x16,
    kAdc2OffsetCalibration0 = 0x17,
    kAdc2OffsetCalibration1 = 0x18,
    kAdc2FullScaleCalibration0 = 0x19,
    kAdc2FullScaleCalibration02 = 0x1A,
} Register;

typedef enum {
    kNoOpertion = 0x00,
    kReset = 0x06,
    kStartAdc1Conversation = 0x08,
    kStopAdc1Conversation = 0x0A,
    kStartAdc2Conversation = 0x0C,
    kStopAdc2Conversation = 0x0E,
    kReadAdc1Data = 0x12,
    kReadAdc2Data = 0x14,
    kAdc1SystemCalibrateOffset = 0x16,
    kAdc1SystemCalibrateGain = 0x17,
    kAdc1SystemCalibrateInternalOffset = 0x19,
    kAdc2SystemCalibrateOffset = 0x1B,
    kAdc2SystemCalibrateGain = 0x1C,
    kAdc2SystemCalibrateInternalOffset = 0x1E,
    kReadRegisters = 0x20,
    kWriteRegisters = 0x40,
} Command;

class ADS1263 {
 public:

    ADS1263();
    virtual ~ADS1263();

    void Init(SSI::SsiBase base);

    void SetupMux(ADCNumber adc, ADCChannel postive_input,
                  ADCChannel negative_input);
    void SetupADC1Filter(Filter filter);
    void SetupADC1SampleRate(ADC1DataRate datarate);
    void SetupADC2SampleRate(ADC2DataRate datarate);
    void SetupCurrentSource(IDAC::Setup *setup);
    void StartConversation(ADCNumber adc);
    void StopConversation(ADCNumber adc);
    void StartSystemOffsetCalibration(ADCNumber adc);
    void StartSelfOffsetCalibration(ADCNumber adc);
    void StartSystemFullScaleCalibration(ADCNumber adc);
    void Reset();
    bool ReadADCValues(int32_t *adc1data, int32_t *adc2data);
    bool ReadADC1Values(int32_t *adc1data);
    bool ReadADC2Values(int32_t *adc2data);
    uint8_t ReadID();

 private:

    bool WriteSingleRegister(Register reg, uint8_t data);
    uint8_t ReadSingleRegister(Register reg);
    bool ReadADCData(ADCNumber adc);
    void SendCommand(Command command);

    ADC1DataRate adc1datarate;
    ADC2DataRate adc2datarate;
    Filter adc1filter;
    ADCChannel adc1_positiv_input;
    ADCChannel adc1_negative_input;
    ADCChannel adc2_positiv_input;
    ADCChannel adc2_negative_input;
    IDAC::Setup idac1_setup;
    IDAC::Setup idac2_setup;
    int32_t adc1data;
    int32_t adc2data;

    SSI::SsiBase ssibase;
    Pin ssi_cs;
};

}  // namespace ADS1263

#endif /* ADS1263_HPP_ */
