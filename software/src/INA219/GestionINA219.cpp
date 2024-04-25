/*
 * GestionINA219.cpp
 *
 *  Created on: 25 september 2017
 *      Author: totof
 * Controle un module INA219
 */
 
#include "GestionINA219.h"

// *****************
// Constructeur vide
// *****************
GestionINA219::GestionINA219() {
}

// ******************************
// Constructeur
// @Param l'adresse I2C du module
// ******************************
GestionINA219::GestionINA219(Address address) {
	init(address);
}

// *******************************
// Initialisation
// @Param l'adresse I2C du module
// @Return true si ok, false sinon
// *******************************
bool GestionINA219::init(Address address) {
	GestionI2C::init(address);
	return GestionI2C::isInitialize();
}

// ***********************************************
// Reset du module
// Remet tous les registres a la valeur par defaut
// @Return true si ok, false sinon
// ***********************************************
bool GestionINA219::reset(void) {
	return !write16Swapped(Config, Reset);	
}

// *******************************
// Configuration
// @Param BusVoltageRange
// @Param Gain
// @Param BusADCResolution
// @Param ShuntADCResolution
// @Param Mode
// @Return true si ok, false sinon
// *******************************
bool GestionINA219::config(BusVoltageRange busVoltageRange, Gain gain, BusADCResolution busADCResolution, ShuntADCResolution shuntADCResolution, Mode mode) {
	  return !write16Swapped(Config, busVoltageRange | gain | busADCResolution | shuntADCResolution | mode);
}

// *******************************
// Calibration de base a 3.2A
// @Param tension (16 ou 32V)
// @Return true si ok, false sinon
// *******************************
bool GestionINA219::setCalibration_3_2A(BusVoltageRange tensionMax, BusADCResolution busADCResolution, ShuntADCResolution shuntADCResolution, Mode mode) {
	// VSHUNT_MAX = 0.32          (Gain 8, 320mV)
  
	// 1. Courant max possible
	// MaxPossible_I = VSHUNT_MAX / ResistanceShunt
	// MaxPossible_I = 3.2A
  
	// 2. Determine courant max attendu
	// MaxExpected_I = 3.2A
  
	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0,0000976592              (97uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0,0007812500              (781uA per bit)
  
	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.0001 (100uA per bit)
	
	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * ResistanceShunt))
	// Cal = 4096 (0x1000)
	  
	// calibration = 4096;
	
	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.002 (2mW per bit)
	  
	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 3.2767A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.32767V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If
	  
	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 3.2 * 32V
	// MaximumPower = 102.4W
	  
	// Set multipliers to convert raw current/power values
	currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
	powerDivider_mW = 2;     // Power LSB = 2mW per bit (20 * current LSB)

	// Set Calibration register to 'calibration' calculated above	
	bool result1 = !write16Swapped(Calibration, 4096);
	bool result2 = config(tensionMax, _8_320mV, busADCResolution, shuntADCResolution, mode);
	return result1 & result2;
}

// *******************************
// Calibration de base a 1.6A
// @Param tension (16 ou 32V)
// @Return true si ok, false sinon
// *******************************
bool GestionINA219::setCalibration_1_6A(BusVoltageRange tensionMax, BusADCResolution busADCResolution, ShuntADCResolution shuntADCResolution, Mode mode) {
	// VSHUNT_MAX = 0.16          (Gain 4, 160mV)
  
	// 1. Courant max possible
	// MaxPossible_I = VSHUNT_MAX / ResistanceShunt
	// MaxPossible_I = 1.6A
  
	// 2. Determine courant max attendu
	// MaxExpected_I = 1.6A
  
	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0,0000488296              (49uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0,0003906250              (390uA per bit)
  
	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0,00005 (50uA per bit)
	
	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * ResistanceShunt))
	// Cal = 8192 (0x2000)
	  
	// calibration = 8192;
	
	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.001 (1mW per bit)
	  
	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 1.63835A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.163835V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If
	  
	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 1.6 * 32V
	// MaximumPower = 51.2W
	  
	// Set multipliers to convert raw current/power values
	currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
	powerDivider_mW = 1;     // Power LSB = 1mW per bit (20 * currentLSB)

	// Set Calibration register to 'calibration' calculated above	
	bool result1 = !write16Swapped(Calibration, 8192);
	bool result2 = config(tensionMax, _4_160mV, busADCResolution, shuntADCResolution, mode);
	return result1 & result2;
}

// *******************************
// Calibration de base a 0.8A
// @Param tension (16 ou 32V)
// @Return true si ok, false sinon
// *******************************
bool GestionINA219::setCalibration_0_8A(BusVoltageRange tensionMax, BusADCResolution busADCResolution, ShuntADCResolution shuntADCResolution, Mode mode) {
	// VSHUNT_MAX = 0.08          (Gain 2, 80mV)
  
	// 1. Courant max possible
	// MaxPossible_I = VSHUNT_MAX / ResistanceShunt
	// MaxPossible_I = 0.8A
  
	// 2. Determine courant max attendu
	// MaxExpected_I = 0.8A
  
	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0,0000244148              (24uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0,0001953125              (195uA per bit)
  
	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0,00002 (20uA per bit)
	
	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * ResistanceShunt))
	// Cal = 20480 (0x5000)
	  
	// calibration = 20480;
	
	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.0004 (400uW per bit)
	  
	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 0.65534A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.065534V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If
	  
	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 0.65534 * 32V
	// MaximumPower = 20.97W
	  
	// Set multipliers to convert raw current/power values
	currentDivider_mA = 50;  // Current LSB = 20uA per bit (1000/20 = 50)
	powerDivider_mW = 2.5;     // Power LSB = 400uW per bit (20 * currentLSB)

	// Set Calibration register to 'calibration' calculated above	
	bool result1 = !write16Swapped(Calibration, 20480);
	bool result2 = config(tensionMax, _2_80mV, busADCResolution, shuntADCResolution, mode);
	return result1 & result2;
}

// *******************************
// Calibration de base a 0.4A
// @Param tension (16 ou 32V)
// @Return true si ok, false sinon
// *******************************
bool GestionINA219::setCalibration_0_4A(BusVoltageRange tensionMax, BusADCResolution busADCResolution, ShuntADCResolution shuntADCResolution, Mode mode) {
	// VSHUNT_MAX = 0.04          (Gain 1, 40mV)
  
	// 1. Courant max possible
	// MaxPossible_I = VSHUNT_MAX / ResistanceShunt
	// MaxPossible_I = 0.4A
  
	// 2. Determine courant max attendu
	// MaxExpected_I = 0.4A
  
	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0,0000122074              (12uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0,0000976563              (98uA per bit)
  
	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.00001 (10uA per bit)
	
	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * ResistanceShunt))
	// Cal = 4096 (0x1000)
	  
	// calibration = 40960;
	
	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.002 (200uW per bit)
	  
	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 0.32767A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.032767V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If
	  
	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 0.32767 * 32V
	// MaximumPower = 10.48W
	  
	// Set multipliers to convert raw current/power values
	currentDivider_mA = 100;  // Current LSB = 10uA per bit (1000/10 = 100)
	powerDivider_mW = 5;     // Power LSB = 200uW per bit  (20 * currentLSB)
	// Set Calibration register to 'calibration' calculated above	
	bool result1 = !write16Swapped(Calibration, 40960);
	bool result2 = config(tensionMax, _1_40mV, busADCResolution, shuntADCResolution, mode);
	return result1 & result2;
}

// ***************************
// Donne le Voltage sur le bus
// @Return Voltage du bus
// ***************************
float GestionINA219::getBusVoltage_V(void) {
	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	return ((read16Swapped(BusVoltage) >> 3) * 4) * 0.001;
}

// *****************************
// Donne le Voltage sur le shunt
// @Return Voltage du bus
// *****************************
float GestionINA219::getShuntVoltage_mV(void) {
	return read16Swapped(ShuntVoltage) * 0.01;
}

// ******************
// Donne le courant
// @Return le courant
// ******************
float GestionINA219::getCurrent_mA(void) {
	return read16Swapped(Current) / currentDivider_mA;
}

// ********************
// Donne la puissance
// @Return la puissance
// ********************
float GestionINA219::getPower_W(void) {
	return read16Swapped(Power) / powerDivider_mW / 1000.0;
}

// **************************************************************
// Indique si les conversion sont OK, donc les registres lisibles
// True si oui, false sinon
// **************************************************************
bool GestionINA219::isConversionOk(void) {
	return read16Swapped(BusVoltage) & Conversion;
}

// *******************************************************************************
// Indique si les registres sont en overflow, donc les mesures ne sont pas fiables
// True si oui, false sinon
// *******************************************************************************
bool GestionINA219::isOverflow(void) {
	return read16Swapped(BusVoltage) & Overflow;
}
