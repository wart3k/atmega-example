/*
 * Timer.h
 *
 * Created: 19.10.2025 22:30:00
 * Author: WART3K
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "Clock.h"

// ============================================
// Timer-Konfiguration Enumerationen
// ============================================

enum class TimerPrescaler {
    DIV1 = 0,
    DIV2 = 1,
    DIV4 = 2,
    DIV8 = 3,
    DIV16 = 4,
    DIV64 = 5,
    DIV256 = 6,
    DIV1024 = 7
};

enum class TimerMode {
    PERIODIC,       // Periodischer Interrupt
    TIMEOUT         // Einmaliger Timeout
};

// ============================================
// Timer-Wrapper-Structs
// ============================================

struct TimerB0_Timer {
    static constexpr volatile TCB_t* timer = &TCB0;
    static constexpr uint8_t timerIndex = 0;
};

struct TimerB1_Timer {
    static constexpr volatile TCB_t* timer = &TCB1;
    static constexpr uint8_t timerIndex = 1;
};

struct TimerB2_Timer {
    static constexpr volatile TCB_t* timer = &TCB2;
    static constexpr uint8_t timerIndex = 2;
};

// ============================================
// Timer Callbacks
// ============================================

namespace TimerCallbacks {
    extern void (*callback0)();
    extern void (*callback1)();
    extern void (*callback2)();
}

// ============================================
// Timer Template-Klasse
// ============================================

template<typename TIMER>
class Timer {
private:
    static bool initialized;
    static volatile uint32_t millisCounter;
    static volatile uint32_t microsCounter;
    
    static inline void (**getCallback())() {
        switch(TIMER::timerIndex) {
            case 0: return &TimerCallbacks::callback0;
            case 1: return &TimerCallbacks::callback1;
            case 2: return &TimerCallbacks::callback2;
            default: return nullptr;
        }
    }
    
public:
    // ========================================
    // Basis Timer-Funktionen
    // ========================================
    
    // Timer für periodischen Interrupt initialisieren (in Mikrosekunden)
    static void begin(uint32_t microseconds, void (*callback)()) {
        // Callback registrieren
        auto timerCallback = getCallback();
        if (timerCallback != nullptr) {
            *timerCallback = callback;
        }
        
        // Timer stoppen
        TIMER::timer->CTRLA = 0;
        
        // Periodic Interrupt Mode
        TIMER::timer->CTRLB = TCB_CNTMODE_INT_gc;
        
        // Compare Value berechnen
        uint32_t clkFreq = Clock::getFrequency();
        uint32_t compareValue = (clkFreq / 1000000UL) * microseconds;
        
        // Prescaler bestimmen wenn nötig
        uint8_t prescaler = 0;
        while (compareValue > 65535 && prescaler < 2) {
            prescaler++;
            compareValue >>= 1;
        }
        
        if (compareValue > 65535) compareValue = 65535;
        
        TIMER::timer->CCMP = compareValue;
        
        // Interrupt aktivieren
        TIMER::timer->INTCTRL = TCB_CAPT_bm;
        
        // Timer mit Prescaler starten
        TIMER::timer->CTRLA = (prescaler << TCB_CLKSEL_gp) | TCB_ENABLE_bm;
        
        millisCounter = 0;
        microsCounter = 0;
        initialized = true;
    }
    
    // Timer für Millisekunden-Interrupt
    static void beginMillis(uint32_t milliseconds, void (*callback)()) {
        begin(milliseconds * 1000UL, callback);
    }
    
    // Timer beenden
    static void end() {
        TIMER::timer->CTRLA &= ~TCB_ENABLE_bm;
        TIMER::timer->INTCTRL = 0;
        initialized = false;
    }
    
    // ========================================
    // Control-Funktionen
    // ========================================
    
    // Timer starten
    static void start() {
        TIMER::timer->CTRLA |= TCB_ENABLE_bm;
    }
    
    // Timer stoppen
    static void stop() {
        TIMER::timer->CTRLA &= ~TCB_ENABLE_bm;
    }
    
    // Timer zurücksetzen
    static void reset() {
        TIMER::timer->CNT = 0;
        millisCounter = 0;
        microsCounter = 0;
    }
    
    // ========================================
    // Interrupt Handler
    // ========================================
    
    static void handleInterrupt() {
        // Flag löschen
        TIMER::timer->INTFLAGS = TCB_CAPT_bm;
        
        // Counter erhöhen
        millisCounter++;
        microsCounter += 1000;  // Annahme: 1ms Timer
        
        // User-Callback aufrufen
        auto callback = getCallback();
        if (callback != nullptr && *callback != nullptr) {
            (*callback)();
        }
    }
    
    // ========================================
    // Utility-Funktionen
    // ========================================
    
    // Millisekunden seit Start
    static uint32_t millis() {
        return millisCounter;
    }
    
    // Mikrosekunden seit Start
    static uint32_t micros() {
        return microsCounter;
    }
    
    // Counter-Wert lesen
    static uint16_t getCount() {
        return TIMER::timer->CNT;
    }
};

// ============================================
// Template statische Member-Definitionen
// ============================================

template<typename TIMER>
bool Timer<TIMER>::initialized = false;

template<typename TIMER>
volatile uint32_t Timer<TIMER>::millisCounter = 0;

template<typename TIMER>
volatile uint32_t Timer<TIMER>::microsCounter = 0;

#endif //__TIMER_H__