/*
 * GestionINA219.h
 *
 *  Created on: 25 september 2017
 *      Author: totof
 * Controle un module INA219
 */

#ifndef _GESTION_INA219_H_
#define _GESTION_INA219_H_

#include <stdint.h>
#include "GestionI2C.h"

#define ResistanceShunt 0.1

enum Address {
	ADDR_40 = 0x40,
	ADDR_41 = 0x41,
	ADDR_44 = 0x44,
	ADDR_45 = 0x45
};

enum Register {
	Config = 0x00,
	ShuntVoltage = 0x01,
	BusVoltage = 0x02,
	Power = 0x03,
	Current = 0x04,
	Calibration = 0x05
};

//enum Mask {
//	BusVoltageRange = 0x2000,
 //   Gain = 0x1800,
 //   BusADCResolution = 0x0780,
  //  ShuntADCResolution = 0x0078,
  //  Mode = 0x0007
//};

enum BusVoltageRange {
	_16V = 0x0000,  // 0-16V Range
    _32V = 0x2000  // 0-32V Range
};
	
enum Gain {
	_1_40mV = 0x0000,  // Gain 1, 40mV Range
    _2_80mV = 0x0800,  // Gain 2, 80mV Range
    _4_160mV = 0x1000,  // Gain 4, 160mV Range
    _8_320mV = 0x1800  // Gain 8, 320mV Range
};

enum BusADCResolution {	
    B_9Bits_1S_84US = 0x0000,  // 1 x 9-bit Bus sample
    B_10Bits_1S_148US = 0x0080,  // 1 x 10-bit Bus sample
    B_11Bits_1S_276US = 0x0100,  // 1 x 11-bit Bus sample
    B_12Bits_1S_532US = 0x0180,  // 1 x 12-bit Bus sample
    B_12Bits_2S_1060US = 0x0480,	 // 2 x 12-bit Bus samples averaged together
    B_12Bits_4S_2130US = 0x0500,  // 4 x 12-bit Bus samples averaged together
    B_12Bits_8S_4260US = 0x0580,  // 8 x 12-bit Bus samples averaged together
    B_12Bits_16S_8510US = 0x0600,  // 16 x 12-bit Bus samples averaged together
    B_12Bits_32S_17MS = 0x0680,  // 32 x 12-bit Bus samples averaged together
    B_12Bits_64S_34MS = 0x0700,  // 64 x 12-bit Bus samples averaged together
    B_12Bits_128S_69MS = 0x0780  // 128 x 12-bit Bus samples averaged together
};

enum ShuntADCResolution {
	S_9Bits_1S_84US = 0x0000,  // 1 x 9-bit shunt sample
    S_10Bits_1S_148US = 0x0008,  // 1 x 10-bit shunt sample
    S_11Bits_1S_276US = 0x0010,  // 1 x 11-bit shunt sample
    S_12Bits_1S_532US = 0x0018,  // 1 x 12-bit shunt sample
    S_12Bits_2S_1060US = 0x0048,	 // 2 x 12-bit shunt samples averaged together
    S_12Bits_4S_2130US = 0x0050,  // 4 x 12-bit shunt samples averaged together
    S_12Bits_8S_4260US = 0x0058,  // 8 x 12-bit shunt samples averaged together
    S_12Bits_16S_8510US = 0x0060,  // 16 x 12-bit shunt samples averaged together
    S_12Bits_32S_17MS = 0x0068,  // 32 x 12-bit shunt samples averaged together
    S_12Bits_64S_34MS = 0x0070,  // 64 x 12-bit shunt samples averaged together
    S_12Bits_128S_69MS = 0x0078  // 128 x 12-bit shunt samples averaged together
};
	
enum Mode {
    PowerDown = 0x0000,
    ShuntVoltageTriggered = 0x0001,
    BusVoltageTrigerred = 0x0002,
    ShuntAndBusVoltageTriggered = 0x0003,
    ADCOff = 0x0004,
    ShuntVoltageContinuous = 0x0005,
    BusVoltageContinuous = 0x0006,
    ShuntAndBusVoltageContinuous = 0x0007
};

enum FlagMask {
	Overflow = 0x0001,
	Conversion = 0x0002
};

enum Config {
	Reset = 0x8000  // Reset Bit
};

class GestionINA219 : GestionI2C {
	public:
		GestionINA219();
		GestionINA219(Address);
		bool init(Address);
		bool setCalibration_3_2A(BusVoltageRange busVoltageRange, BusADCResolution busADCResolution = B_12Bits_1S_532US, ShuntADCResolution shuntADCResolution = S_12Bits_1S_532US, Mode mode = ShuntAndBusVoltageContinuous);
		bool setCalibration_1_6A(BusVoltageRange busVoltageRange, BusADCResolution busADCResolution = B_12Bits_1S_532US, ShuntADCResolution shuntADCResolution = S_12Bits_1S_532US, Mode mode = ShuntAndBusVoltageContinuous);
		bool setCalibration_0_8A(BusVoltageRange busVoltageRange, BusADCResolution busADCResolution = B_12Bits_1S_532US, ShuntADCResolution shuntADCResolution = S_12Bits_1S_532US, Mode mode = ShuntAndBusVoltageContinuous);
		bool setCalibration_0_4A(BusVoltageRange busVoltageRange, BusADCResolution busADCResolution = B_12Bits_1S_532US, ShuntADCResolution shuntADCResolution = S_12Bits_1S_532US, Mode mode = ShuntAndBusVoltageContinuous);
		float getBusVoltage_V(void);
		float getShuntVoltage_mV(void);
		float getCurrent_mA(void);
		float getPower_W(void);
		bool reset(void);
		bool isConversionOk(void);
		bool isOverflow(void);

	private:
		// The following multipliers are used to convert raw current and power
		// values to mA and mW, taking into account the current config settings
		float currentDivider_mA;
		float powerDivider_mW;
  
		bool config(BusVoltageRange, Gain, BusADCResolution, ShuntADCResolution, Mode);
};

#endif