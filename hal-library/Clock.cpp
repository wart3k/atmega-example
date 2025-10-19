/*
 * Clock.cpp
 *
 * Created: 19.10.2025 20:09:41
 *  Author: WART3K
 */ 

#include "Clock.h"

// ============================================
// Statische Member-Variable definieren
// ============================================

uint32_t Clock::currentFrequency = 20000000UL;  // Default: 20 MHz

// ============================================
// Implementierungen
// ============================================

void Clock::begin(ClockSource source) {
	// Configuration Change Protection (CCP) - erlaubt Schreiben in geschützte Register
	CPU_CCP = CCP_IOREG_gc;
	
	switch(source) {
		case ClockSource::INTERNAL_20MHz:
		// Standard - 20 MHz interner Oszillator, kein Prescaler
		CLKCTRL.MCLKCTRLB = 0;  // Prescaler disabled
		currentFrequency = 20000000UL;
		break;
		
		case ClockSource::INTERNAL_16MHz:
		// 20 MHz / 2 = 10 MHz
		// Hinweis: ATmega4808 kann nicht direkt 16 MHz
		CPU_CCP = CCP_IOREG_gc;
		CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;
		currentFrequency = 10000000UL;
		break;
		
		case ClockSource::INTERNAL_10MHz:
		// 20 MHz / 2 = 10 MHz
		CPU_CCP = CCP_IOREG_gc;
		CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;
		currentFrequency = 10000000UL;
		break;
		
		case ClockSource::INTERNAL_5MHz:
		// 20 MHz / 4 = 5 MHz
		CPU_CCP = CCP_IOREG_gc;
		CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
		currentFrequency = 5000000UL;
		break;
		
		case ClockSource::EXTERNAL_CRYSTAL:
		// Externer Quarz (erfordert Hardware-Setup)
		// WARNUNG: Nur verwenden, wenn externer Quarz angeschlossen ist!
		CPU_CCP = CCP_IOREG_gc;
		CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_EXTCLK_gc;
		// Frequenz muss manuell gesetzt werden
		currentFrequency = 16000000UL;  // Beispiel: 16 MHz Quarz
		break;
	}
}

uint32_t Clock::getFrequency() {
	return currentFrequency;
}

void Clock::setPrescaler(uint8_t prescaler) {
	uint8_t prescalerBits = 0;
	uint32_t baseFreq = 20000000UL;  // Basis: 20 MHz interner Oszillator
	
	switch(prescaler) {
		case 1:
		prescalerBits = 0;  // Kein Prescaler
		currentFrequency = baseFreq;
		break;
		case 2:
		prescalerBits = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;
		currentFrequency = baseFreq / 2;
		break;
		case 4:
		prescalerBits = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
		currentFrequency = baseFreq / 4;
		break;
		case 8:
		prescalerBits = CLKCTRL_PDIV_8X_gc | CLKCTRL_PEN_bm;
		currentFrequency = baseFreq / 8;
		break;
		case 16:
		prescalerBits = CLKCTRL_PDIV_16X_gc | CLKCTRL_PEN_bm;
		currentFrequency = baseFreq / 16;
		break;
		case 32:
		prescalerBits = CLKCTRL_PDIV_32X_gc | CLKCTRL_PEN_bm;
		currentFrequency = baseFreq / 32;
		break;
		case 64:
		prescalerBits = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm;
		currentFrequency = baseFreq / 64;
		break;
		default:
		return;  // Ungültiger Prescaler
	}
	
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = prescalerBits;
}

void Clock::lock() {
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKLOCK = CLKCTRL_LOCKEN_bm;
}