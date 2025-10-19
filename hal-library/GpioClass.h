/*
 * GpioClass.h
 *
 * Created: 19.10.2025 18:44:33
 *  Author: WART3K
 */ 


#ifndef GPIOCLASS_H_
#define GPIOCLASS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ============================================
// Enumerationen
// ============================================

enum class PinMode {
	INPUT,
	OUTPUT
};

enum class PinState {
	LOW,
	HIGH
};

enum class InterruptMode {
	DISABLED,           // Kein Interrupt
	BOTH_EDGES,         // Steigende und fallende Flanke
	RISING_EDGE,        // Nur steigende Flanke
	FALLING_EDGE,       // Nur fallende Flanke
	LOW_LEVEL           // Low-Level
};

// ============================================
// Port-Wrapper-Structs
// ============================================

struct PortA {
	static constexpr volatile PORT_t* port = &PORTA;
	static constexpr uint8_t portIndex = 0;
};

struct PortB {
	static constexpr volatile PORT_t* port = &PORTB;
	static constexpr uint8_t portIndex = 1;
};

struct PortC {
	static constexpr volatile PORT_t* port = &PORTC;
	static constexpr uint8_t portIndex = 2;
};

struct PortD {
	static constexpr volatile PORT_t* port = &PORTD;
	static constexpr uint8_t portIndex = 3;
};

struct PortE {
	static constexpr volatile PORT_t* port = &PORTE;
	static constexpr uint8_t portIndex = 4;
};

struct PortF {
	static constexpr volatile PORT_t* port = &PORTF;
	static constexpr uint8_t portIndex = 5;
};

// ============================================
// Globale Callback-Arrays für Interrupts
// ============================================

namespace InterruptCallbacks {
	extern void (*portA[8])();
	extern void (*portB[8])();
	extern void (*portC[8])();
	extern void (*portD[8])();
	extern void (*portE[4])();
	extern void (*portF[7])();
}

// ============================================
// GPIO Template-Klasse
// ============================================

template<typename PORT, uint8_t PIN>
class GPIO {
	private:
	static constexpr uint8_t pin_bm = (1 << PIN);
	
	// Port-spezifisches Callback-Array ermitteln
	static inline void (**getCallbackArray())() {
		switch(PORT::portIndex) {
			case 0: return InterruptCallbacks::portA;
			case 1: return InterruptCallbacks::portB;
			case 2: return InterruptCallbacks::portC;
			case 3: return InterruptCallbacks::portD;
			case 4: return InterruptCallbacks::portE;
			case 5: return InterruptCallbacks::portF;
			default: return nullptr;
		}
	}
	
	public:
	// ========================================
	// Basis GPIO-Funktionen
	// ========================================
	
	// Pin-Modus setzen (INPUT oder OUTPUT)
	static void setMode(PinMode mode) {
		if (mode == PinMode::OUTPUT) {
			PORT::port->DIR |= pin_bm;
			} else {
			PORT::port->DIR &= ~pin_bm;
		}
	}
	
	// Pin auf HIGH oder LOW setzen
	static void write(PinState state) {
		if (state == PinState::HIGH) {
			PORT::port->OUT |= pin_bm;
			} else {
			PORT::port->OUT &= ~pin_bm;
		}
	}
	
	// Pin umschalten (toggle)
	static void toggle() {
		PORT::port->IN |= pin_bm;
	}
	
	// Pin-Zustand lesen
	static bool read() {
		return (PORT::port->IN & pin_bm) != 0;
	}
	
	// ========================================
	// Pull-up Funktionen
	// ========================================
	
	// Pull-up aktivieren (nur im INPUT-Modus sinnvoll)
	static void enablePullup() {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		*pinctrl |= PORT_PULLUPEN_bm;
	}
	
	// Pull-up deaktivieren
	static void disablePullup() {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		*pinctrl &= ~PORT_PULLUPEN_bm;
	}
	
	// ========================================
	// Interrupt-Funktionen
	// ========================================
	
	// Interrupt konfigurieren und Callback registrieren
	static void attachInterrupt(InterruptMode mode, void (*userCallback)()) {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		
		// Callback speichern
		auto callbacks = getCallbackArray();
		if (callbacks != nullptr) {
			callbacks[PIN] = userCallback;
		}
		
		// Alte ISC-Bits löschen
		*pinctrl &= ~PORT_ISC_gm;
		
		// Neue ISC-Bits setzen
		switch(mode) {
			case InterruptMode::BOTH_EDGES:
			*pinctrl |= PORT_ISC_BOTHEDGES_gc;
			break;
			case InterruptMode::RISING_EDGE:
			*pinctrl |= PORT_ISC_RISING_gc;
			break;
			case InterruptMode::FALLING_EDGE:
			*pinctrl |= PORT_ISC_FALLING_gc;
			break;
			case InterruptMode::LOW_LEVEL:
			*pinctrl |= PORT_ISC_LEVEL_gc;
			break;
			case InterruptMode::DISABLED:
			default:
			*pinctrl |= PORT_ISC_INTDISABLE_gc;
			break;
		}
	}
	
	// Interrupt deaktivieren
	static void detachInterrupt() {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		*pinctrl &= ~PORT_ISC_gm;
		*pinctrl |= PORT_ISC_INTDISABLE_gc;
		
		auto callbacks = getCallbackArray();
		if (callbacks != nullptr) {
			callbacks[PIN] = nullptr;
		}
	}
	
	// Interrupt-Flag manuell löschen
	static void clearInterruptFlag() {
		PORT::port->INTFLAGS = pin_bm;
	}
	
	// ========================================
	// Erweiterte Pin-Konfiguration
	// ========================================
	
	// Inverted I/O aktivieren
	static void enableInvert() {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		*pinctrl |= PORT_INVEN_bm;
	}
	
	// Inverted I/O deaktivieren
	static void disableInvert() {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		*pinctrl &= ~PORT_INVEN_bm;
	}
	
	// Input Disable (nützlich für analoge Pins)
	static void disableDigitalInput() {
		volatile uint8_t* pinctrl = &PORT::port->PIN0CTRL + PIN;
		*pinctrl |= PORT_ISC_INPUT_DISABLE_gc;
	}
};

// ============================================
// Helper-Funktionen für ISRs
// ============================================

namespace GPIO_Internal {
	// Generische ISR-Handler-Funktion
	inline void handlePortInterrupt(volatile PORT_t* port, void (**callbacks)(), uint8_t maxPins) {
		uint8_t flags = port->INTFLAGS;
		
		for (uint8_t i = 0; i < maxPins; i++) {
			if (flags & (1 << i)) {
				if (callbacks[i] != nullptr) {
					callbacks[i]();
				}
				port->INTFLAGS = (1 << i);  // Flag löschen
			}
		}
	}
}

#endif /* GPIOCLASS_H_ */