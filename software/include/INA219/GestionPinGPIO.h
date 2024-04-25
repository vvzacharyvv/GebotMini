/*
 * GestionPinGPIO.h
 *
 *  Created on: 1 août 2016
 *      Author: totof
 */

#ifndef GESTIONPINGPIO_H_
#define GESTIONPINGPIO_H_

#include <wiringPi.h>
#include "GestionPin.h"

// Type fonction d'interruption
typedef void (*interrupt)(void);

// Liste des pin
enum PinGPIO {
  GPIO02 = 2,
  GPIO03 = 3,
  GPIO04 = 4,
  GPIO17 = 17,
  GPIO27 = 27,
  GPIO22 = 22,
  GPIO10 = 10,
  GPIO09 = 9,
  GPIO11 = 11,
  GPIO05 = 5,
  GPIO06 = 6,
  GPIO13 = 13,
  GPIO19 = 19,
  GPIO26 = 26,
  GPIO14 = 14,
  GPIO15 = 15,
  GPIO18 = 18,
  GPIO23 = 23,
  GPIO24 = 24,
  GPIO25 = 25,
  GPIO08 = 8,
  GPIO07 = 7,
  GPIO12 = 12,
  GPIO16 = 16,
  GPIO20 = 20,
  GPIO21 = 21
};

// Liste des directions
enum PinDirection {
	In = INPUT,
	Out = OUTPUT
};

// Liste des fronts d'interruption
enum FrontIntr {
	Falling = INT_EDGE_FALLING,
	Rising = INT_EDGE_RISING,
	Both = INT_EDGE_BOTH,
	Setup = INT_EDGE_SETUP	
};

// Liste des résistances de rappel
enum Pud {
	Off = PUD_OFF,
	Down = PUD_DOWN,
	Up = PUD_UP
};

// Liste des niveaux
enum Level {
	Low = LOW,
	High = HIGH
};

// Classe de gestion des GPIO
class GestionPinGPIO : public GestionPin{
public:
	GestionPinGPIO();
	GestionPinGPIO(PinGPIO);
	void init(PinGPIO);
	void in(void);
	void out(void);
	void pullUp(void);
	void pullDn(void);
	void pullOff(void);
	void write(Level);
	void off(void);
	void on(void);
	void invertState(void);
	Level read(void);
	bool isOn(void);
	bool isOff(void);
	int fctInterrupt(FrontIntr, interrupt);
	bool isToDesactivate(void);
	void toDesactivate(void);
	void noDesactivate(void);
	virtual ~GestionPinGPIO();

private:
	int pinNumber;
	void pinModePI(PinDirection);
	void pullUpDnControlPI(Pud);
	void setToDesactivate(bool);
	bool desactivateFlag = true;
};

#endif /* GESTIONPINGPIO_H_ */
