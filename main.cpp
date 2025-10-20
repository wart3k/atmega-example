#define F_CPU 20000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "hal-library/Clock.h"
#include "hal-library/GpioClass.h"
#include "hal-library/Usart.h"

using LED = GPIO<PortA, 7>;
using Serial = UART<Usart0>;

int main() {
	// Clock initialisieren
	Clock::begin(ClockSource::INTERNAL_20MHz);
	
	// LED für Debug
	LED::setMode(PinMode::OUTPUT);
	LED::write(PinState::HIGH);  // LED an = Programm gestartet
	
	// UART initialisieren
	Serial::begin(115200);
	
	LED::write(PinState::LOW);  // LED aus = UART initialisiert
	
	// Globale Interrupts
	sei();
	
	// Test: Erste Nachricht senden
	LED::write(PinState::HIGH);  // LED an = vor write()
	
	Serial::write('H');  // Wenn es hier hängt, ist TX-Pin nicht richtig
	
	LED::write(PinState::LOW);  // LED aus = write() erfolgreich
	
	// Wenn wir hier ankommen, funktioniert write()!
	const char* msg = "UART OK!\r\n";
	for (uint8_t i = 0; msg[i] != '\0'; i++) {
		Serial::write(msg[i]);
	}
	
	while(1) {
		// Echo-Test
		if (Serial::available()) {
			uint8_t data = Serial::read();
			Serial::write(data);
			LED::toggle();
		}
	}
	
	return 0;
}