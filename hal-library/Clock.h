/*
 * Clock.h
 *
 * Created: 19.10.2025 19:59:06
 *  Author: WART3K
 */ 


#ifndef CLOCK_H_
#define CLOCK_H_

#include <avr/io.h>
#include <stdint.h>

// ============================================
// Clock-Frequenz Enumerationen
// ============================================

enum class ClockSource {
	INTERNAL_20MHz,     // Interner 20 MHz Oszillator (Standard)
	INTERNAL_16MHz,     // Interner 20 MHz mit Prescaler (tatsächlich 10 MHz)
	INTERNAL_10MHz,     // Interner 20 MHz mit Prescaler /2 = 10 MHz
	INTERNAL_5MHz,      // Interner 20 MHz mit Prescaler /4 = 5 MHz
	EXTERNAL_CRYSTAL    // Externer Quarz (muss hardware-seitig vorhanden sein)
};

// ============================================
// Clock Management Klasse - NUR DEKLARATION
// ============================================

class Clock {
	private:
	static uint32_t currentFrequency;
	
	public:
	// Clock initialisieren und konfigurieren
	static void begin(ClockSource source = ClockSource::INTERNAL_20MHz);
	
	// Aktuelle CPU-Frequenz abfragen
	static uint32_t getFrequency();
	
	// Prescaler setzen (1, 2, 4, 8, 16, 32, 64)
	static void setPrescaler(uint8_t prescaler);
	
	// Clock-Lock aktivieren (verhindert weitere Änderungen)
	static void lock();
};

// ============================================
// F_CPU für delay.h definieren
// ============================================
#ifndef F_CPU
#define F_CPU 20000000UL  // Default: 20 MHz
#endif


#endif /* CLOCK_H_ */