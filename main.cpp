/*
 * example-project.cpp
 *
 * Created: 19.10.2025 18:31:03
 * Author : WART3K
 */ 


#define F_CPU 20000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "hal-library/Clock.h"
#include "hal-library/GpioClass.h"
#include "hal-library/Usart.h"

// ============================================
// Pin-Definitionen
// ============================================

using LED = GPIO<PortA, 7>;

// UART-Instanzen
using Serial = UART<Usart0>;    // USART0: TX=PA0, RX=PA1
using Serial1 = UART<Usart1>;   // USART1: TX=PC0, RX=PC1
using Serial2 = UART<Usart2>;   // USART2: TX=PF0, RX=PF1

// ============================================
// UART RX Callback
// ============================================

void onSerialReceive() {
	// Wird aufgerufen, wenn Daten empfangen wurden
	LED::toggle();
}

// ============================================
// Main-Funktion
// ============================================

int main() {
	// Clock initialisieren
	Clock::begin(ClockSource::INTERNAL_20MHz);
	
	// LED konfigurieren
	LED::setMode(PinMode::OUTPUT);
	
	// UART initialisieren (115200 Baud, 8N1)
	Serial::begin(115200);
	
	// Optional: RX Callback registrieren
	Serial::attachRxInterrupt(onSerialReceive);
	
	// Globale Interrupts aktivieren
	sei();
	
	// Startmeldung
	Serial::println("ATmega4808 UART Test");
	Serial::println("Ready!");
	
	uint32_t counter = 0;
	
	while(1) {
		// Empfangene Daten verarbeiten
		if (Serial::available()) {
			uint8_t data = Serial::read();
			
			// Echo zurücksenden
			Serial::print("Received: ");
			Serial::print((char)data);
			Serial::print(" (0x");
			Serial::printHex(data);
			Serial::println(")");
		}
		
		// Periodisch Status ausgeben
		Serial::print("Counter: ");
		Serial::println(counter++);
		
		LED::toggle();
		_delay_ms(1000);
	}
	
	return 0;
}