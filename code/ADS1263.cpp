/*
 * ADS1263.cpp
 *
 *  Created on: 23.08.2017
 *      Author: rico
 */

#include "ADS1263.hpp"

ADS1263::ADS1263::ADS1263() {
  idac1_setup.channel = IDAC::kAIN0;
  idac2_setup.channel = IDAC::kAIN1;

  idac1_setup.magnitude = IDAC::k00_00_uA;
  idac1_setup.magnitude = IDAC::k00_00_uA;

  idac1_setup.module = IDAC::kIDACModule1;
  idac2_setup.module = IDAC::kIDACModule2;

  adc1datarate = k1200_SPS;
  adc2datarate = k100_SPS_2;
}

ADS1263::ADS1263::~ADS1263() {
}

void ADS1263::ADS1263::Init(SSI::SsiBase base) {
  ssibase = base;
  switch (ssibase) {
    case SSI::kSSI0_BASE: {
      //GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
      GPIOPinConfigure(GPIO_PA2_SSI0CLK);
      GPIOPinConfigure(GPIO_PA3_SSI0FSS);
      GPIOPinConfigure(GPIO_PA4_SSI0RX);
      GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_3);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
      while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)) {
      }

    }
      break;
    case SSI::kSSI1_BASE: {
      GPIOPinConfigure(GPIO_PF2_SSI1CLK);
      GPIOPinConfigure(GPIO_PF3_SSI1FSS);
      GPIOPinConfigure(GPIO_PF0_SSI1RX);
      GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_3);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
      while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1)) {
      }

    }
      break;
    case SSI::kSSI2_BASE: {
      ssi_cs.number = GPIO_PIN_5;
      ssi_cs.periph = SYSCTL_PERIPH_GPIOB;
      ssi_cs.port = GPIO_PORTB_BASE;
      SysCtlPeripheralEnable(ssi_cs.periph);
      while (!SysCtlPeripheralReady(ssi_cs.periph)) {
      }
      GPIOPinConfigure(GPIO_PB4_SSI2CLK);
      //GPIOPinConfigure(GPIO_PB5_SSI2FSS);
      GPIOPinTypeGPIOOutput(ssi_cs.port, ssi_cs.number);
      GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
      GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
      GPIOPinConfigure(GPIO_PB6_SSI2RX);
      GPIOPinConfigure(GPIO_PB7_SSI2TX);
      GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
      while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2)) {
      }
    }
      break;
    case SSI::kSSI3_BASE: {
      GPIOPinConfigure(GPIO_PD0_SSI3CLK);
      GPIOPinConfigure(GPIO_PD1_SSI3FSS);
      GPIOPinConfigure(GPIO_PD2_SSI3RX);
      GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_1);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
      while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)) {
      }
    }
      break;
  }
  SSIConfigSetExpClk(ssibase, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 250000, 8);
  SSIEnable(ssibase);
  SysCtlDelay(10000);

}

void ADS1263::ADS1263::SetupMux(ADCNumber adc, ADCChannel postive_input, ADCChannel negative_input) {
  switch (adc) {
    case kADC1:
      WriteSingleRegister(kInputMux, ((postive_input << 4) | negative_input));
      break;
    case kADC2:
      WriteSingleRegister(kAdc2Mux, ((postive_input << 4) | negative_input));
      break;
    default:
      break;
  }
}

void ADS1263::ADS1263::SetupADC1Filter(Filter filter) {
  WriteSingleRegister(kMode1, filter << 5);
}

void ADS1263::ADS1263::SetupADC1SampleRate(ADC1DataRate datarate) {
  WriteSingleRegister(kMode2, datarate);
}

void ADS1263::ADS1263::SetupADC2SampleRate(ADC2DataRate datarate) {
  WriteSingleRegister(kAdc2Configuration, datarate);
}

void ADS1263::ADS1263::SetupCurrentSource(IDAC::Setup* setup) {
  switch (setup->module) {
    case IDAC::kIDACModule1:
      idac1_setup = *setup;
      break;
    case IDAC::kIDACModule2:
      idac2_setup = *setup;
      break;
    default:
      break;
  }
  WriteSingleRegister(kIdacMux, (idac1_setup.channel | (idac2_setup.channel << 4)));
  WriteSingleRegister(kIdacMagnitude, (idac1_setup.magnitude | (idac2_setup.magnitude << 4)));
}

void ADS1263::ADS1263::StartConversation(ADCNumber adc) {
  if (adc == kADC1) {
    SendCommand(kStartAdc1Conversation);
  } else if (adc == kADC2) {
    SendCommand(kStartAdc2Conversation);
  }
}

void ADS1263::ADS1263::StopConversation(ADCNumber adc) {
  if (adc == kADC1) {
    SendCommand(kStopAdc1Conversation);
  } else if (adc == kADC2) {
    SendCommand(kStopAdc2Conversation);
  }
}

void ADS1263::ADS1263::StartSystemOffsetCalibration(ADCNumber adc) {
  if (adc == kADC1) {
    SendCommand(kAdc1SystemCalibrateOffset);
  } else if (adc == kADC2) {
    SendCommand(kAdc2SystemCalibrateOffset);
  }
}

void ADS1263::ADS1263::StartSelfOffsetCalibration(ADCNumber adc) {
  if (adc == kADC1) {
    SendCommand(kAdc1SystemCalibrateInternalOffset);
  } else if (adc == kADC2) {
    SendCommand(kAdc2SystemCalibrateInternalOffset);
  }
}

void ADS1263::ADS1263::StartSystemFullScaleCalibration(ADCNumber adc) {
  if (adc == kADC1) {
    SendCommand(kAdc1SystemCalibrateGain);
  } else if (adc == kADC2) {
    SendCommand(kAdc2SystemCalibrateGain);
  }
}

void ADS1263::ADS1263::Reset() {
  SendCommand(kReset);
}

bool ADS1263::ADS1263::ReadADCValues(int32_t* adc1data, int32_t* adc2data) {

  bool returnvalue;
  returnvalue = ReadADCData(kADC1);
  *adc1data = this->adc1data;

  if (returnvalue) {
    returnvalue = ReadADCData(kADC2);
    *adc2data = this->adc2data;
    return returnvalue;
  } else {
    return false;
  }
}

bool ADS1263::ADS1263::ReadADC1Values(int32_t* adc1data) {
  bool returnvalue;
  returnvalue = ReadADCData(kADC1);
  *adc1data = this->adc1data;
  return returnvalue;
}

bool ADS1263::ADS1263::ReadADC2Values(int32_t* adc2data) {
  bool returnvalue;
  returnvalue = ReadADCData(kADC2);
  *adc2data = this->adc2data;
  return returnvalue;
}

uint8_t ADS1263::ADS1263::ReadID() {
  return ReadSingleRegister(kId);
}

bool ADS1263::ADS1263::WriteSingleRegister(Register reg, uint8_t data) {
  uint32_t txdata[3] = { (kWriteRegisters | reg), 0x00, data };

  GPIOPinWrite(ssi_cs.port, ssi_cs.number, 0);
  for (uint8_t x = 0; x < 3; x++) {
    SSIDataPut(ssibase, txdata[x]);
  }
  while(SSIBusy(ssibase)){}
  GPIOPinWrite(ssi_cs.port, ssi_cs.number, ssi_cs.number);

  if (data == ReadSingleRegister(reg)) {
    return true;
  } else {
    return false;
  }
}

uint8_t ADS1263::ADS1263::ReadSingleRegister(Register reg) {

  uint32_t rxdata[16];
  uint32_t txdata[3] = { (kReadRegisters | reg), 0x00, 0x00 };

  SSIDataGetNonBlocking(ssibase, rxdata);
  GPIOPinWrite(ssi_cs.port, ssi_cs.number, 0);

  for (uint8_t x = 0; x < 3; x++) {
    SSIDataPut(ssibase, txdata[x]);
  }

  while(SSIBusy(ssibase)){}
  GPIOPinWrite(ssi_cs.port, ssi_cs.number, ssi_cs.number);

  for (uint8_t x = 0; x < 3; x++) {
    SSIDataGet(ssibase, &rxdata[x]);
  }

  return rxdata[2];
}

bool ADS1263::ADS1263::ReadADCData(ADCNumber adc) {
  if (adc == kADC1) {

    uint32_t rxdata[16] = {0};

    SSIDataGetNonBlocking(ssibase, rxdata);
    GPIOPinWrite(ssi_cs.port, ssi_cs.number, 0);

    for (uint8_t x = 0; x < 6; x++) {
      SSIDataPut(ssibase, 0x00);
    }

    while(SSIBusy(ssibase)){}
    GPIOPinWrite(ssi_cs.port, ssi_cs.number, ssi_cs.number);

    for (uint8_t x = 0; x < 6; x++) {
      SSIDataGet(ssibase, &rxdata[x]);
    }

    adc1data = ((rxdata[1] << 24) | (rxdata[2] << 16) | (rxdata[3] << 8) | rxdata[4]);
    return true;

  } else if (adc == kADC2) {
    uint32_t rxdata[16];

    SSIDataGetNonBlocking(ssibase, rxdata);
    GPIOPinWrite(ssi_cs.port, ssi_cs.number, 0);

    SSIDataPut(ssibase, kReadAdc2Data);
    for (uint8_t x = 0; x < 6; x++) {
      SSIDataPut(ssibase, 0x00);
    }

    while(SSIBusy(ssibase)){}
    GPIOPinWrite(ssi_cs.port, ssi_cs.number, ssi_cs.number);

    for (uint8_t x = 0; x < 7; x++) {
      SSIDataGet(ssibase, &rxdata[x]);
    }

    adc2data = ((rxdata[1] << 24) | (rxdata[2] << 16) | (rxdata[3] << 8) | rxdata[4]);
    return true;

  } else {
    return false;
  }
}

void ADS1263::ADS1263::SendCommand(Command command) {

  GPIOPinWrite(ssi_cs.port, ssi_cs.number, 0);
  SSIDataPut(ssibase, command);
  while(SSIBusy(ssibase)){}
  GPIOPinWrite(ssi_cs.port, ssi_cs.number, ssi_cs.number);
}
