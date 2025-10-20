#include "ADC.h"

// ============================================
// Callback-Funktionen initialisieren
// ============================================

namespace ADCCallbacks {
	void (*conversionCompleteCallback)() = nullptr;
}

// ============================================
// ISR-Definition für ADC0
// ============================================

// ADC0 Result Ready Interrupt
ISR(ADC0_RESRDY_vect) {
	ADC<Adc0>::handleInterrupt();
}