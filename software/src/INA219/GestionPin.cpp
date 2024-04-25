/*
 * GestionPin.cpp
 *
 *  Created on: 1 août 2016
 *      Author: totof
 */

#include <wiringPi.h>
#include "GestionPin.h"

bool GestionPin::setup = false;

// ****************************************
// Constructeur
// Permet d'appeler une seule fois le setup
// ****************************************
GestionPin::GestionPin() {
	if(!setup) {
		wiringPiSetupGpio();
	}
	setup = true;
}

// **********************************************************
// Destructeur
// **********************************************************
GestionPin::~GestionPin() {
}
