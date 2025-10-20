
/*
 * PWM.h
 *
 * Created: 19.10.2025 22:15:00
 * Author: WART3K
 */

#ifndef __PWM_H__
#define __PWM_H__

#include <avr/io.h>
#include <stdint.h>

#include "Clock.h"

// ============================================
// PWM-Konfiguration Enumerationen
// ============================================

enum class PWMPrescaler {
    DIV1 = 0,       // Kein Prescaler
    DIV2 = 1,       // CLK/2
    DIV4 = 2,       // CLK/4
    DIV8 = 3,       // CLK/8
    DIV16 = 4,      // CLK/16
    DIV64 = 5,      // CLK/64
    DIV256 = 6,     // CLK/256
    DIV1024 = 7     // CLK/1024
};

enum class PWMMode {
    NORMAL,         // Normal PWM
    FREQUENCY,      // Frequency PWM
    SINGLE_SLOPE,   // Single Slope PWM
    DUAL_SLOPE      // Dual Slope PWM
};

// ============================================
// PWM-Timer-Wrapper-Structs
// ============================================

// TCA0 - 16-Bit Timer mit 3 Compare Channels
struct TimerA0 {
    static constexpr volatile TCA_SINGLE_t* timer = &TCA0.SINGLE;
    static constexpr uint8_t timerIndex = 0;
    
    // PWM Output Pins
    // WO0 = PA0, WO1 = PA1, WO2 = PA2
    // WO3 = PA3, WO4 = PA4, WO5 = PA5
    
    static inline void configurePins(uint8_t channels) {
        if (channels & (1 << 0)) PORTA.DIRSET = PIN0_bm;  // WO0
        if (channels & (1 << 1)) PORTA.DIRSET = PIN1_bm;  // WO1
        if (channels & (1 << 2)) PORTA.DIRSET = PIN2_bm;  // WO2
        if (channels & (1 << 3)) PORTA.DIRSET = PIN3_bm;  // WO3
        if (channels & (1 << 4)) PORTA.DIRSET = PIN4_bm;  // WO4
        if (channels & (1 << 5)) PORTA.DIRSET = PIN5_bm;  // WO5
    }
};

// TCB0 - 16-Bit Timer (einzelner PWM-Kanal)
struct TimerB0 {
    static constexpr volatile TCB_t* timer = &TCB0;
    static constexpr uint8_t timerIndex = 1;
    
    // PWM Output: PA2
    static inline void configurePins() {
        PORTA.DIRSET = PIN2_bm;
    }
};

struct TimerB1 {
    static constexpr volatile TCB_t* timer = &TCB1;
    static constexpr uint8_t timerIndex = 2;
    
    // PWM Output: PA3
    static inline void configurePins() {
        PORTA.DIRSET = PIN3_bm;
    }
};

// ============================================
// PWM Template-Klasse (für TCA)
// ============================================

template<typename TIMER>
class PWM {
private:
    static bool initialized;
    static uint16_t periodValue;
    
public:
    // ========================================
    // Basis PWM-Funktionen (TCA0)
    // ========================================
    
    // PWM initialisieren
    static void begin(uint32_t frequency = 1000, 
                      PWMPrescaler prescaler = PWMPrescaler::DIV1) {
        
        // Timer stoppen
        TIMER::timer->CTRLA = 0;
        
        // Prescaler setzen
        TIMER::timer->CTRLA = static_cast<uint8_t>(prescaler) << TCA_SINGLE_CLKSEL_gp;
        
        // Period berechnen
        uint32_t clkFreq = Clock::getFrequency();
        uint32_t prescalerValue = 1 << static_cast<uint8_t>(prescaler);
        periodValue = (clkFreq / prescalerValue / frequency) - 1;
        
        if (periodValue > 65535) periodValue = 65535;
        
        TIMER::timer->PER = periodValue;
        
        // Single Slope PWM Mode
        TIMER::timer->CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
        
        // Compare Channels initial auf 0
        TIMER::timer->CMP0 = 0;
        TIMER::timer->CMP1 = 0;
        TIMER::timer->CMP2 = 0;
        
        // Timer starten
        TIMER::timer->CTRLA |= TCA_SINGLE_ENABLE_bm;
        
        initialized = true;
    }
    
    // PWM beenden
    static void end() {
        TIMER::timer->CTRLA &= ~TCA_SINGLE_ENABLE_bm;
        TIMER::timer->CTRLB = 0;
        initialized = false;
    }
    
    // ========================================
    // Channel-Funktionen
    // ========================================
    
    // Channel aktivieren (0, 1, 2)
    static void enableChannel(uint8_t channel) {
        if (channel > 2) return;
        
        // Pins konfigurieren
        uint8_t channelMask = (1 << channel);
        TIMER::configurePins(channelMask);
        
        // Compare Output aktivieren
        TIMER::timer->CTRLB |= (TCA_SINGLE_CMP0EN_bm << channel);
    }
    
    // Channel deaktivieren
    static void disableChannel(uint8_t channel) {
        if (channel > 2) return;
        TIMER::timer->CTRLB &= ~(TCA_SINGLE_CMP0EN_bm << channel);
    }
    
    // Duty Cycle setzen (0-255, 8-Bit)
    static void write(uint8_t channel, uint8_t dutyCycle) {
        if (channel > 2 || !initialized) return;
        
        // 8-Bit auf 16-Bit Period skalieren
        uint16_t compareValue = ((uint32_t)dutyCycle * periodValue) / 255;
        
        switch(channel) {
            case 0:
                TIMER::timer->CMP0BUF = compareValue;
                break;
            case 1:
                TIMER::timer->CMP1BUF = compareValue;
                break;
            case 2:
                TIMER::timer->CMP2BUF = compareValue;
                break;
        }
    }
    
    // Duty Cycle setzen (0-65535, 16-Bit)
    static void write16(uint8_t channel, uint16_t dutyCycle) {
        if (channel > 2 || !initialized) return;
        
        // 16-Bit auf Period skalieren
        uint16_t compareValue = ((uint32_t)dutyCycle * periodValue) / 65535;
        
        switch(channel) {
            case 0:
                TIMER::timer->CMP0BUF = compareValue;
                break;
            case 1:
                TIMER::timer->CMP1BUF = compareValue;
                break;
            case 2:
                TIMER::timer->CMP2BUF = compareValue;
                break;
        }
    }
    
    // Prozent setzen (0.0 - 100.0)
    static void writePercent(uint8_t channel, float percent) {
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        uint8_t dutyCycle = (uint8_t)(percent * 2.55f);
        write(channel, dutyCycle);
    }
    
    // ========================================
    // Frequenz ändern
    // ========================================
    
    static void setFrequency(uint32_t frequency) {
        uint32_t clkFreq = Clock::getFrequency();
        
        // Aktuellen Prescaler ermitteln
        uint8_t prescalerBits = (TIMER::timer->CTRLA >> TCA_SINGLE_CLKSEL_gp) & 0x07;
        uint32_t prescalerValue = 1 << prescalerBits;
        
        periodValue = (clkFreq / prescalerValue / frequency) - 1;
        if (periodValue > 65535) periodValue = 65535;
        
        TIMER::timer->PERBUF = periodValue;
    }
    
    // Aktuelle Frequenz abfragen
    static uint32_t getFrequency() {
        uint32_t clkFreq = Clock::getFrequency();
        uint8_t prescalerBits = (TIMER::timer->CTRLA >> TCA_SINGLE_CLKSEL_gp) & 0x07;
        uint32_t prescalerValue = 1 << prescalerBits;
        return clkFreq / prescalerValue / (periodValue + 1);
    }
};

// ============================================
// Vereinfachte PWM-Klasse für TCB
// ============================================

template<typename TIMER>
class SimplePWM {
private:
    static bool initialized;
    
public:
    // PWM initialisieren (TCB)
    static void begin(uint32_t frequency = 1000) {
        // TCB in 8-Bit PWM Mode
        TIMER::timer->CTRLB = TCB_CNTMODE_PWM8_gc;
        
        // Period setzen (8-Bit)
        TIMER::timer->CCMPL = 255;
        
        // Initial Duty Cycle = 0
        TIMER::timer->CCMPH = 0;
        
        // Pin konfigurieren
        TIMER::configurePins();
        
        // Output Enable
        TIMER::timer->CTRLB |= TCB_CCMPEN_bm;
        
        // Timer starten
        TIMER::timer->CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
        
        initialized = true;
    }
    
    // PWM beenden
    static void end() {
        TIMER::timer->CTRLA &= ~TCB_ENABLE_bm;
        initialized = false;
    }
    
    // Duty Cycle setzen (0-255)
    static void write(uint8_t dutyCycle) {
        if (!initialized) return;
        TIMER::timer->CCMPH = dutyCycle;
    }
    
    // Prozent setzen
    static void writePercent(float percent) {
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        uint8_t dutyCycle = (uint8_t)(percent * 2.55f);
        write(dutyCycle);
    }
};

// ============================================
// Template statische Member-Definitionen
// ============================================

template<typename TIMER>
bool PWM<TIMER>::initialized = false;

template<typename TIMER>
uint16_t PWM<TIMER>::periodValue = 0;

template<typename TIMER>
bool SimplePWM<TIMER>::initialized = false;

#endif //__PWM_H__