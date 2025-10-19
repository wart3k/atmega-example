/* 
* Usart.h
*
* Created: 19.10.2025 20:15:04
* Author: WART3K
*/


#ifndef __USART_H__
#define __USART_H__

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "Clock.h"

// ============================================
// UART-Konfiguration Enumerationen
// ============================================

enum class UartParity {
	NONE,       // Keine Parität
	EVEN,       // Gerade Parität
	ODD         // Ungerade Parität
};

enum class UartStopBits {
	ONE,        // 1 Stop-Bit
	TWO         // 2 Stop-Bits
};

enum class UartDataBits {
	BITS_5,     // 5 Datenbits
	BITS_6,     // 6 Datenbits
	BITS_7,     // 7 Datenbits
	BITS_8,     // 8 Datenbits (Standard)
	BITS_9      // 9 Datenbits
};

// ============================================
// USART-Wrapper-Structs
// ============================================

struct Usart0 {
	static constexpr volatile USART_t* usart = &USART0;
	static constexpr uint8_t usartIndex = 0;
};

struct Usart1 {
	static constexpr volatile USART_t* usart = &USART1;
	static constexpr uint8_t usartIndex = 1;
};

struct Usart2 {
	static constexpr volatile USART_t* usart = &USART2;
	static constexpr uint8_t usartIndex = 2;
};

// ============================================
// Callback-Funktionstypen
// ============================================

namespace UartCallbacks {
	extern void (*rxCallback0)();
	extern void (*rxCallback1)();
	extern void (*rxCallback2)();
}

// ============================================
// UART Template-Klasse - NUR DEKLARATION
// ============================================

template<typename USART>
class UART {
	private:
	static constexpr uint8_t RX_BUFFER_SIZE = 64;
	static constexpr uint8_t TX_BUFFER_SIZE = 64;
	
	// Ring-Buffer für RX
	static volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
	static volatile uint8_t rxHead;
	static volatile uint8_t rxTail;
	
	// Ring-Buffer für TX
	static volatile uint8_t txBuffer[TX_BUFFER_SIZE];
	static volatile uint8_t txHead;
	static volatile uint8_t txTail;
	
	// Baud-Rate berechnen
	static inline uint16_t calculateBaud(uint32_t baudrate, uint32_t cpuFreq);
	
	// Callback-Array ermitteln
	static inline void (**getRxCallback())();
	
	public:
	// ========================================
	// Basis UART-Funktionen
	// ========================================
	
	// UART initialisieren
	static void begin(uint32_t baudrate,
	UartDataBits dataBits = UartDataBits::BITS_8,
	UartParity parity = UartParity::NONE,
	UartStopBits stopBits = UartStopBits::ONE);
	
	// UART beenden
	static void end();
	
	// ========================================
	// Sende-Funktionen (Blocking)
	// ========================================
	
	// Einzelnes Byte senden
	static void write(uint8_t data);
	
	// Array senden
	static void write(const uint8_t* data, uint16_t length);
	
	// String senden (C-String)
	static void print(const char* str);
	
	// String mit Newline senden
	static void println(const char* str);
	
	// Einzelnes Zeichen senden
	static void print(char c);
	
	// Zahl senden (Dezimal)
	static void print(int32_t number);
	
	// Zahl mit Newline senden
	static void println(int32_t number);
	
	// Hexadezimal ausgeben
	static void printHex(uint8_t value);
	static void printHex(uint16_t value);
	static void printHex(uint32_t value);
	
	// ========================================
	// Empfangs-Funktionen
	// ========================================
	
	// Bytes verfügbar?
	static uint8_t available();
	
	// Einzelnes Byte lesen
	static uint8_t read();
	
	// Byte lesen ohne aus Buffer zu entfernen
	static uint8_t peek();
	
	// Buffer leeren
	static void flush();
	
	// ========================================
	// Interrupt-Funktionen
	// ========================================
	
	// RX Interrupt Callback registrieren
	static void attachRxInterrupt(void (*callback)());
	
	// RX Interrupt deaktivieren
	static void detachRxInterrupt();
	
	// Wird von ISR aufgerufen
	static void handleRxInterrupt();
	static void handleTxInterrupt();
};

// ============================================
// Template statische Member-Definitionen
// ============================================

template<typename USART>
volatile uint8_t UART<USART>::rxBuffer[RX_BUFFER_SIZE];

template<typename USART>
volatile uint8_t UART<USART>::rxHead = 0;

template<typename USART>
volatile uint8_t UART<USART>::rxTail = 0;

template<typename USART>
volatile uint8_t UART<USART>::txBuffer[TX_BUFFER_SIZE];

template<typename USART>
volatile uint8_t UART<USART>::txHead = 0;

template<typename USART>
volatile uint8_t UART<USART>::txTail = 0;

// ============================================
// Template-Implementierungen (müssen im Header bleiben!)
// ============================================

template<typename USART>
inline uint16_t UART<USART>::calculateBaud(uint32_t baudrate, uint32_t cpuFreq) {
	// Formel: BAUD = (64 * f_CLK_PER) / (S * f_BAUD)
	// S = 16 für normale Geschwindigkeit
	return (uint16_t)((cpuFreq * 64UL) / (16UL * baudrate));
}

template<typename USART>
inline void (**UART<USART>::getRxCallback())() {
	if (USART::usartIndex == 0) return &UartCallbacks::rxCallback0;
	if (USART::usartIndex == 1) return &UartCallbacks::rxCallback1;
	if (USART::usartIndex == 2) return &UartCallbacks::rxCallback2;
	return nullptr;
}

template<typename USART>
void UART<USART>::begin(uint32_t baudrate,
UartDataBits dataBits,
UartParity parity,
UartStopBits stopBits) {
	// Baud-Rate setzen (Standard: 20 MHz CPU)
	uint16_t baud = calculateBaud(baudrate, Clock::getFrequency());
	USART::usart->BAUD = baud;
	
	// Frame-Format konfigurieren
	uint8_t ctrlc = 0;
	
	// Datenbits
	switch(dataBits) {
		case UartDataBits::BITS_5:
		ctrlc |= USART_CHSIZE_5BIT_gc;
		break;
		case UartDataBits::BITS_6:
		ctrlc |= USART_CHSIZE_6BIT_gc;
		break;
		case UartDataBits::BITS_7:
		ctrlc |= USART_CHSIZE_7BIT_gc;
		break;
		case UartDataBits::BITS_9:
		ctrlc |= USART_CHSIZE_9BITL_gc;
		break;
		case UartDataBits::BITS_8:
		default:
		ctrlc |= USART_CHSIZE_8BIT_gc;
		break;
	}
	
	// Parität
	switch(parity) {
		case UartParity::EVEN:
		ctrlc |= USART_PMODE_EVEN_gc;
		break;
		case UartParity::ODD:
		ctrlc |= USART_PMODE_ODD_gc;
		break;
		case UartParity::NONE:
		default:
		ctrlc |= USART_PMODE_DISABLED_gc;
		break;
	}
	
	// Stop-Bits
	if (stopBits == UartStopBits::TWO) {
		ctrlc |= USART_SBMODE_2BIT_gc;
		} else {
		ctrlc |= USART_SBMODE_1BIT_gc;
	}
	
	USART::usart->CTRLC = ctrlc;
	
	// TX und RX aktivieren
	USART::usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	
	// RX Complete Interrupt aktivieren
	USART::usart->CTRLA |= USART_RXCIE_bm;
	
	// Buffer zurücksetzen
	rxHead = 0;
	rxTail = 0;
	txHead = 0;
	txTail = 0;
}

template<typename USART>
void UART<USART>::end() {
	// TX und RX deaktivieren
	USART::usart->CTRLB &= ~(USART_TXEN_bm | USART_RXEN_bm);
	
	// Interrupts deaktivieren
	USART::usart->CTRLA &= ~(USART_RXCIE_bm | USART_TXCIE_bm);
	
	// Buffer leeren
	rxHead = 0;
	rxTail = 0;
	txHead = 0;
	txTail = 0;
}

template<typename USART>
void UART<USART>::write(uint8_t data) {
	// Warten bis Transmit-Buffer leer ist
	while (!(USART::usart->STATUS & USART_DREIF_bm));
	
	// Daten senden
	USART::usart->TXDATAL = data;
}

template<typename USART>
void UART<USART>::write(const uint8_t* data, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		write(data[i]);
	}
}

template<typename USART>
void UART<USART>::print(const char* str) {
	while (*str) {
		write(*str++);
	}
}

template<typename USART>
void UART<USART>::println(const char* str) {
	print(str);
	write('\r');
	write('\n');
}

template<typename USART>
void UART<USART>::print(char c) {
	write(c);
}

template<typename USART>
void UART<USART>::print(int32_t number) {
	if (number < 0) {
		write('-');
		number = -number;
	}
	
	// Zahl in String umwandeln (rekursiv)
	char buffer[12];  // Max: -2147483648 = 11 Zeichen + \0
	uint8_t i = 0;
	
	if (number == 0) {
		write('0');
		return;
	}
	
	while (number > 0) {
		buffer[i++] = '0' + (number % 10);
		number /= 10;
	}
	
	// Rückwärts ausgeben
	while (i > 0) {
		write(buffer[--i]);
	}
}

template<typename USART>
void UART<USART>::println(int32_t number) {
	print(number);
	write('\r');
	write('\n');
}

template<typename USART>
void UART<USART>::printHex(uint8_t value) {
	const char hexChars[] = "0123456789ABCDEF";
	write(hexChars[(value >> 4) & 0x0F]);
	write(hexChars[value & 0x0F]);
}

template<typename USART>
void UART<USART>::printHex(uint16_t value) {
	printHex((uint8_t)(value >> 8));
	printHex((uint8_t)(value & 0xFF));
}

template<typename USART>
void UART<USART>::printHex(uint32_t value) {
	printHex((uint16_t)(value >> 16));
	printHex((uint16_t)(value & 0xFFFF));
}

template<typename USART>
uint8_t UART<USART>::available() {
	return (RX_BUFFER_SIZE + rxHead - rxTail) % RX_BUFFER_SIZE;
}

template<typename USART>
uint8_t UART<USART>::read() {
	// Warten bis Daten verfügbar
	if (rxHead == rxTail) {
		return 0;
	}
	
	uint8_t data = rxBuffer[rxTail];
	rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
	return data;
}

template<typename USART>
uint8_t UART<USART>::peek() {
	if (rxHead == rxTail) {
		return 0;
	}
	return rxBuffer[rxTail];
}

template<typename USART>
void UART<USART>::flush() {
	rxHead = 0;
	rxTail = 0;
}

template<typename USART>
void UART<USART>::attachRxInterrupt(void (*callback)()) {
	auto rxCallback = getRxCallback();
	if (rxCallback != nullptr) {
		*rxCallback = callback;
	}
}

template<typename USART>
void UART<USART>::detachRxInterrupt() {
	auto rxCallback = getRxCallback();
	if (rxCallback != nullptr) {
		*rxCallback = nullptr;
	}
}

template<typename USART>
void UART<USART>::handleRxInterrupt() {
	// Daten aus Hardware-Register lesen
	uint8_t data = USART::usart->RXDATAL;
	
	// In Ring-Buffer speichern
	uint8_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
	
	// Buffer-Overflow vermeiden
	if (nextHead != rxTail) {
		rxBuffer[rxHead] = data;
		rxHead = nextHead;
	}
	
	// User-Callback aufrufen
	auto callback = getRxCallback();
	if (callback != nullptr && *callback != nullptr) {
		(*callback)();
	}
}

template<typename USART>
void UART<USART>::handleTxInterrupt() {
	// TODO: Implementierung für buffered TX
}

#endif //__USART_H__
