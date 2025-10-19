/*
 * GpioClass.cpp
 *
 * Created: 19.10.2025 19:21:53
 *  Author: WART3K
 */ 

#include "GpioClass.h"

namespace InterruptCallbacks {
	void (*portA[8])() = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
	void (*portB[8])() = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
	void (*portC[8])() = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
	void (*portD[8])() = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
	void (*portE[4])() = {nullptr, nullptr, nullptr, nullptr};
	void (*portF[7])() = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
}

ISR(PORTA_PORT_vect) {
	GPIO_Internal::handlePortInterrupt(&PORTA, InterruptCallbacks::portA, 8);
}

ISR(PORTB_PORT_vect) {
	GPIO_Internal::handlePortInterrupt(&PORTB, InterruptCallbacks::portB, 8);
}

ISR(PORTC_PORT_vect) {
	GPIO_Internal::handlePortInterrupt(&PORTC, InterruptCallbacks::portC, 8);
}

ISR(PORTD_PORT_vect) {
	GPIO_Internal::handlePortInterrupt(&PORTD, InterruptCallbacks::portD, 8);
}

ISR(PORTE_PORT_vect) {
	GPIO_Internal::handlePortInterrupt(&PORTE, InterruptCallbacks::portE, 4);
}

ISR(PORTF_PORT_vect) {
	GPIO_Internal::handlePortInterrupt(&PORTF, InterruptCallbacks::portF, 7);
}