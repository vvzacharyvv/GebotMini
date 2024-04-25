/*
 * GestionPinGPIO.cpp
 *
 *  Created on: 1 août 2016
 *      Author: totof
 */

#include <wiringPi.h>
#include "GestionPinGPIO.h"

// ***********************
// Constructeur
// Constructeur par defaut
// ***********************
GestionPinGPIO::GestionPinGPIO() {
}

// ******************************************************
// Méthode init en cas d'appel au constructeur par défaut
// @param numéro de pin
// ******************************************************
void GestionPinGPIO::init(PinGPIO pPinNumber) {
	pinNumber = pPinNumber;	
}

// *********************************
// Constructeur
// Mémorise le numéro de pin utilisé
// @param numéro de pin
// *********************************
GestionPinGPIO::GestionPinGPIO(PinGPIO pPinNumber) {
	pinNumber = pPinNumber;
}

// **********************************
// Applique la direction sur une GPIO
// @param pDirection la direction
// **********************************
void GestionPinGPIO::pinModePI(PinDirection pDirection) {
	pinMode(pinNumber, pDirection);
}

// **************
// GPIO en entrée
// **************
void GestionPinGPIO::in() {
	PinDirection input = In;
	pinMode(pinNumber, input);
}

// **************
// GPIO en sortie
// **************
void GestionPinGPIO::out() {
	PinDirection output = Out;
	pinMode(pinNumber, output);
}

// ***********************************************
// Active/désactive les résistance de pull down/up
// @param le type de résistance
// ***********************************************
void GestionPinGPIO::pullUpDnControlPI(Pud pNiveau) {
	pullUpDnControl(pinNumber, pNiveau);
}

// *******************************
// Active la résistance de pull up
// *******************************
void GestionPinGPIO::pullUp() {
	Pud pudUp = Up;
	pullUpDnControl(pinNumber, pudUp);
}

// *********************************
// Active la résistance de pull down
// *********************************
void GestionPinGPIO::pullDn() {
	Pud pudDown = Down;
	pullUpDnControl(pinNumber, pudDown);
}

// ***************************************
// Désactive la résistance de pull up/down
// ***************************************
void GestionPinGPIO::pullOff() {
	Pud pudOff = Off;
	pullUpDnControl(pinNumber, pudOff);
}

// ****************************
// Applique un niveau à la GPIO 
// @param le niveau
// ****************************
void GestionPinGPIO::write(Level pValue) {
	switch(pValue) {
		case LOW: off(); break;
		case HIGH: on(); break;
	}
}

// ********************************
// Applique un niveau bas à la GPIO 
// ********************************
void GestionPinGPIO::off() {
	Level low = Low;
	digitalWrite(pinNumber, LOW);
}

// *********************************
// Applique un niveau haut à la GPIO 
// *********************************
void GestionPinGPIO::on() {
	Level high = High;
	digitalWrite(pinNumber, HIGH);
}

// ****************************
// inverse le niveau de la GPIO
// ****************************
void GestionPinGPIO::invertState() {
	if(isOn()) {
		off();
	} else {
		on();		
	}
}

// ****************************
// Lecture du niveau de la GPIO
// @return le niveau
// ****************************
Level GestionPinGPIO::read() {
	if(digitalRead(pinNumber)) {
		Level high = High;
		return high;
	} else {
		Level low = Low;
		return low;		
	}
}

// *************************************
// Demande si la GPIO est au niveau haut
// @return true si oui sinon false
// *************************************
bool GestionPinGPIO::isOn() {
	Level high = High;
	if(digitalRead(pinNumber) == high){
		return true;
	}
	return false;
}

// ************************************
// Demande si la GPIO est au niveau bas
// @return true si oui sinon false
// ************************************
bool GestionPinGPIO::isOff() {
	Level low = Low;
	if(digitalRead(pinNumber) == low){
		return true;
	}
	return false;
}

// ***********************************************
// Active une fonction d'interruption sur un front
// @param le front
// @param un pointeur sur la fonction
// @return 0 si OK
// ***********************************************
int GestionPinGPIO::fctInterrupt(FrontIntr pFront, interrupt intr) {
	return wiringPiISR(pinNumber, pFront, *intr);
}

// ******************************************
// Demande si le destructeur sera appliqué
// return destroyFlag, le flag de destructeur
// ******************************************
bool GestionPinGPIO::isToDesactivate(void) {
	return desactivateFlag;
}

// ****************************************
// Indique que le destructeur sera appliqué
// ****************************************
void GestionPinGPIO::toDesactivate(void) {
	setToDesactivate(true);
}

// ***********************************************
// Indique que le destructeur ne sera pas appliqué
// ***********************************************
void GestionPinGPIO::noDesactivate(void) {
	setToDesactivate(false);
}

// ************************************
// Positionne le flag de destructeur
// @param pValue le flag de destructeur
// ************************************
void GestionPinGPIO::setToDesactivate(bool pValue) {
	desactivateFlag = pValue;
}

// **********************************************************
// Destructeur
// Si le flag de destroy est activé,
// met la GPIO en entrée et désactive la résistance de rappel
// sinon, la GPIO reste en état même à la sortie du programme
// **********************************************************
GestionPinGPIO::~GestionPinGPIO() {
	if(isToDesactivate()) {
		Pud pudOff = Off;
		PinDirection in = In;
		pullUpDnControlPI(pudOff);
		pinModePI(in);
	}
}
