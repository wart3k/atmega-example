/* 
* Usart.cpp
*
* Created: 19.10.2025 20:15:04
* Author: WART3K
*/


#include "Usart.h"

// ============================================
// Callback-Funktionen initialisieren
// ============================================

namespace UartCallbacks {
	void (*rxCallback0)() = nullptr;
	void (*rxCallback1)() = nullptr;
	void (*rxCallback2)() = nullptr;
}

// ============================================
// ISR-Definitionen für alle USART-Module
// ============================================

// USART0 RX Complete Interrupt
ISR(USART0_RXC_vect) {
	UART<Usart0>::handleRxInterrupt();
}

// USART1 RX Complete Interrupt
ISR(USART1_RXC_vect) {
	UART<Usart1>::handleRxInterrupt();
}

// USART2 RX Complete Interrupt
ISR(USART2_RXC_vect) {
	UART<Usart2>::handleRxInterrupt();
}

// Optional: TX Complete Interrupts
// ISR(USART0_TXC_vect) {
//     UART<Usart0>::handleTxInterrupt();
// }

// ISR(USART1_TXC_vect) {
//     UART<Usart1>::handleTxInterrupt();
// }

// ISR(USART2_TXC_vect) {
//     UART<Usart2>::handleTxInterrupt();
// }
