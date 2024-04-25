/*
 * GestionI2C.cpp
 *
 * Created on 09/09/2017
 * Author Totof
 *
 */

#include <wiringPiI2C.h>
#include "GestionI2C.h"

// *****************
// Constructeur vide
// *****************
GestionI2C::GestionI2C() {
}

// *************************
// Constructeur
// @param adresse du circuit 
// *************************
GestionI2C::GestionI2C(uint16_t pAddress) {
	init(pAddress);
}

// ******************************************
// Initialisation si constructeur vide appele
// @param adresse du circuit 
// ******************************************
void GestionI2C::init(uint16_t pAddress) {
	handle = wiringPiI2CSetup(pAddress);
	initialize = true;
	if(handle == -1) {
		initialize = false;
	}
}

// ********************************
// L'I2C est t'il initialise
// @return true si oui false si non
// ********************************
bool GestionI2C::isInitialize(void) {
	return initialize;
}
// **************************************************
// Lecture d'une valeur dans un circuit sans registre
// @return valeur lue si positif, erreur si negatif
// **************************************************
int8_t GestionI2C::read8(void) {
	return wiringPiI2CRead (handle);
}

// ***************************************************
// Ecriture d'une valeur dans un circuit sans registre
// @param la valeur a ecrire
// @return true si ok, false si erreur
// ***************************************************
bool GestionI2C::write8(uint8_t data) {
	if(wiringPiI2CWrite (handle, data)) {
		return true;
	} else {
		return false;
	}
}

// ***************************************************
// Lecture d'une valeur dans un circuit avec registres
// @param le registre
// @return valeur lue si positif, erreur si negatif
// ***************************************************
int8_t GestionI2C::read8(uint8_t reg) {
	return wiringPiI2CReadReg8 (handle, reg);
}

// ****************************************************
// Ecriture d'une valeur dans un circuit avec registres
// @param le registre
// @param la valeur a ecrire
// @return true si ok, false si erreur
// ****************************************************
bool GestionI2C::write8(uint8_t reg, uint8_t data) {
	if(wiringPiI2CWriteReg8 (handle, reg, data)) {
		return true;
	} else {
		return false;
	}
}

// ***************************************************
// Lecture d'une valeur dans un circuit avec registres
// @param le registre
// @return valeur lue si positif, erreur si negatif
// ***************************************************
int16_t GestionI2C::read16(uint8_t reg) {
	return wiringPiI2CReadReg16 (handle, reg);
}

// ****************************************************
// Ecriture d'une valeur dans un circuit avec registres
// @param le registre
// @param la valeur a ecrire
// @return true si ok, false si erreur
// ****************************************************
bool GestionI2C::write16(uint8_t reg, uint16_t data) {
	if(wiringPiI2CWriteReg16 (handle, reg, data)) {
		return true;
	} else {
		return false;
	}
}
	
// ***************************************************
// Lecture d'une valeur dans un circuit avec registres
// Les deux octets sont swappe apres lecture
// @param le registre
// @return valeur lue si positif, erreur si negatif
// ***************************************************
int16_t GestionI2C::read16Swapped(uint8_t reg) {
	i2cData data;
	data.uSData = wiringPiI2CReadReg16 (handle, reg);
	swap(data);
	return data.uSData;
}

// ****************************************************
// Ecriture d'une valeur dans un circuit avec registres
// Les deux octets sont swappe avant l'ecriture
// @param le registre
// @param la valeur a ecrire
// @return true si ok, false si erreur
// ****************************************************
bool GestionI2C::write16Swapped(uint8_t reg, uint16_t val) {
	i2cUData data;
	data.uSData = val;
	swapU(data);
	if(wiringPiI2CWriteReg16 (handle, reg, data.uSData)) {
		return true;
	} else {
		return false;
	}
}
	
// *********************************
// swap les deux octets d'un i2cData
// *********************************
void GestionI2C::swap(i2cData &data) {
	int8_t temp = data.uCData[0];
	data.uCData[0] = data.uCData[1];
	data.uCData[1] = temp;
}
	
// **********************************
// swap les deux octets d'un i2cUData
// **********************************
void GestionI2C::swapU(i2cUData &data) {
	uint8_t temp = data.uCData[0];
	data.uCData[0] = data.uCData[1];
	data.uCData[1] = temp;
}
	
// ***********
// Destructeur
// ***********
GestionI2C::~GestionI2C() {
}


