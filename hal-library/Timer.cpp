#include "Timer.h"

// ============================================
// Callback-Funktionen initialisieren
// ============================================

namespace TimerCallbacks {
	void (*callback0)() = nullptr;
	void (*callback1)() = nullptr;
	void (*callback2)() = nullptr;
}

// ============================================
// ISR-Definitionen
// ============================================

ISR(TCB0_INT_vect) {
	Timer<TimerB0_Timer>::handleInterrupt();
}

ISR(TCB1_INT_vect) {
	Timer<TimerB1_Timer>::handleInterrupt();
}

ISR(TCB2_INT_vect) {
	Timer<TimerB2_Timer>::handleInterrupt();
}