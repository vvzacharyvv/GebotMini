/*
 * GestionPinGPIO.h
 *
 *  Created on: 1 août 2016
 *      Author: totof
 */

#ifndef GESTIONPIN_H_
#define GESTIONPIN_H_

#include <wiringPi.h>

// Classe de gestion des pin
class GestionPin {
public:
	GestionPin();
	virtual ~GestionPin();

private:
	static bool setup;
};

#endif /* GESTIONPIN_H_ */
