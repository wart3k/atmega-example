/*
 * ADC.h
 *
 * Created: 19.10.2025 21:30:00
 * Author: WART3K
 */

#ifndef __ADC_H__
#define __ADC_H__

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "Clock.h"

// ============================================
// ADC-Konfiguration Enumerationen
// ============================================

enum class ADCReference {
    VDD,            // VDD als Referenz (Standard)
    INTERNAL_1V1,   // Interne 1.1V Referenz
    INTERNAL_2V5,   // Interne 2.5V Referenz
    INTERNAL_4V3,   // Interne 4.3V Referenz
    EXTERNAL        // Externe Referenz auf VREFA Pin
};

enum class ADCResolution {
    BITS_10,        // 10-Bit Auflösung (0-1023)
    BITS_8          // 8-Bit Auflösung (0-255)
};

enum class ADCPrescaler {
    DIV2,           // CLK/2
    DIV4,           // CLK/4
    DIV8,           // CLK/8
    DIV16,          // CLK/16
    DIV32,          // CLK/32
    DIV64,          // CLK/64
    DIV128,         // CLK/128
    DIV256          // CLK/256
};

enum class ADCSampleDuration {
    SAMPLES_2,      // 2 Samples
    SAMPLES_4,      // 4 Samples
    SAMPLES_8,      // 8 Samples
    SAMPLES_16,     // 16 Samples
    SAMPLES_32,     // 32 Samples
    SAMPLES_64,     // 64 Samples
    SAMPLES_128,    // 128 Samples
    SAMPLES_256     // 256 Samples (höchste Genauigkeit)
};

// ============================================
// ADC-Kanäle (Pins)
// ============================================

enum class ADCChannel {
    AIN0 = 0,       // PD0
    AIN1 = 1,       // PD1
    AIN2 = 2,       // PD2
    AIN3 = 3,       // PD3
    AIN4 = 4,       // PD4
    AIN5 = 5,       // PD5
    AIN6 = 6,       // PD6
    AIN7 = 7,       // PD7
    AIN8 = 8,       // PE0
    AIN9 = 9,       // PE1
    AIN10 = 10,     // PE2
    AIN11 = 11,     // PE3
    AIN12 = 12,     // PF2
    AIN13 = 13,     // PF3
    AIN14 = 14,     // PF4
    AIN15 = 15,     // PF5
    TEMPERATURE = 30,   // Interner Temperatursensor
    GND = 31,           // GND (für Offset-Kalibrierung)
    VDD_DIV10 = 28      // VDD/10 (für VDD-Messung)
};

// ============================================
// ADC-Wrapper-Struct
// ============================================

struct Adc0 {
    static constexpr volatile ADC_t* adc = &ADC0;
    static constexpr uint8_t adcIndex = 0;
};

// ============================================
// Callback-Funktionstyp
// ============================================

namespace ADCCallbacks {
    extern void (*conversionCompleteCallback)();
}

// ============================================
// ADC Template-Klasse
// ============================================

template<typename ADC_MODULE>
class ADC {
private:
    static bool initialized;
    static ADCResolution currentResolution;
    static volatile uint16_t lastResult;
    static volatile bool conversionComplete;
    
    // Callback ermitteln
    static inline void (**getCallback())() {
        return &ADCCallbacks::conversionCompleteCallback;
    }
    
public:
    // ========================================
    // Basis ADC-Funktionen
    // ========================================
    
    // ADC initialisieren
    static void begin(ADCReference reference = ADCReference::VDD,
                      ADCResolution resolution = ADCResolution::BITS_10,
                      ADCPrescaler prescaler = ADCPrescaler::DIV16) {
        
        // Prescaler setzen
        uint8_t prescalerValue = 0;
        switch(prescaler) {
            case ADCPrescaler::DIV2:
                prescalerValue = ADC_PRESC_DIV2_gc;
                break;
            case ADCPrescaler::DIV4:
                prescalerValue = ADC_PRESC_DIV4_gc;
                break;
            case ADCPrescaler::DIV8:
                prescalerValue = ADC_PRESC_DIV8_gc;
                break;
            case ADCPrescaler::DIV16:
                prescalerValue = ADC_PRESC_DIV16_gc;
                break;
            case ADCPrescaler::DIV32:
                prescalerValue = ADC_PRESC_DIV32_gc;
                break;
            case ADCPrescaler::DIV64:
                prescalerValue = ADC_PRESC_DIV64_gc;
                break;
            case ADCPrescaler::DIV128:
                prescalerValue = ADC_PRESC_DIV128_gc;
                break;
            case ADCPrescaler::DIV256:
                prescalerValue = ADC_PRESC_DIV256_gc;
                break;
        }
        
        // Referenzspannung setzen
        uint8_t refValue = 0;
        switch(reference) {
            case ADCReference::INTERNAL_1V1:
                refValue = ADC_REFSEL_INTREF_gc;
                // Interne 1.1V Referenz aktivieren
                VREF.CTRLA |= VREF_ADC0REFSEL_1V5_gc;
                break;
            case ADCReference::INTERNAL_2V5:
                refValue = ADC_REFSEL_INTREF_gc;
                VREF.CTRLA |= VREF_ADC0REFSEL_2V5_gc;
                break;
            case ADCReference::INTERNAL_4V3:
                refValue = ADC_REFSEL_INTREF_gc;
                VREF.CTRLA |= VREF_ADC0REFSEL_4V34_gc;
                break;
            case ADCReference::EXTERNAL:
                refValue = ADC_REFSEL_VREFA_gc;
                break;
            case ADCReference::VDD:
            default:
                refValue = ADC_REFSEL_VDDREF_gc;
                break;
        }
        
        // CTRLC konfigurieren: Prescaler + Referenz
        ADC_MODULE::adc->CTRLC = prescalerValue | refValue;
        
        // Auflösung setzen
        currentResolution = resolution;
        if (resolution == ADCResolution::BITS_8) {
            ADC_MODULE::adc->CTRLA = ADC_RESSEL_8BIT_gc;
        } else {
            ADC_MODULE::adc->CTRLA = ADC_RESSEL_10BIT_gc;
        }
        
        // Sample Duration (Standard: 16 Samples)
        ADC_MODULE::adc->SAMPCTRL = 16;
        
        // ADC aktivieren
        ADC_MODULE::adc->CTRLA |= ADC_ENABLE_bm;
        
        initialized = true;
        conversionComplete = false;
    }
    
    // ADC deaktivieren
    static void end() {
        ADC_MODULE::adc->CTRLA &= ~ADC_ENABLE_bm;
        ADC_MODULE::adc->CTRLA &= ~ADC_RESSEL_bm;
        initialized = false;
    }
    
    // ========================================
    // Lese-Funktionen (Blocking)
    // ========================================
    
    // Kanal lesen (Blocking)
    static uint16_t read(ADCChannel channel) {
        if (!initialized) return 0;
        
        // Kanal auswählen
        ADC_MODULE::adc->MUXPOS = static_cast<uint8_t>(channel);
        
        // Conversion starten
        ADC_MODULE::adc->COMMAND = ADC_STCONV_bm;
        
        // Warten bis fertig
        while (!(ADC_MODULE::adc->INTFLAGS & ADC_RESRDY_bm));
        
        // Flag löschen
        ADC_MODULE::adc->INTFLAGS = ADC_RESRDY_bm;
        
        // Ergebnis lesen
        return ADC_MODULE::adc->RES;
    }
    
    // Mehrere Messungen mitteln
    static uint16_t readAverage(ADCChannel channel, uint8_t samples = 16) {
        if (samples == 0) samples = 1;
        
        uint32_t sum = 0;
        for (uint8_t i = 0; i < samples; i++) {
            sum += read(channel);
        }
        return sum / samples;
    }
    
    // ========================================
    // Interrupt-basierte Messung (Non-Blocking)
    // ========================================
    
    // Conversion starten (Non-Blocking)
    static void startConversion(ADCChannel channel) {
        if (!initialized) return;
        
        conversionComplete = false;
        
        // Kanal auswählen
        ADC_MODULE::adc->MUXPOS = static_cast<uint8_t>(channel);
        
        // Result Ready Interrupt aktivieren
        ADC_MODULE::adc->INTCTRL = ADC_RESRDY_bm;
        
        // Conversion starten
        ADC_MODULE::adc->COMMAND = ADC_STCONV_bm;
    }
    
    // Conversion abgeschlossen?
    static bool isConversionComplete() {
        return conversionComplete;
    }
    
    // Letztes Ergebnis abrufen
    static uint16_t getResult() {
        return lastResult;
    }
    
    // Auf Conversion warten
    static uint16_t waitForResult() {
        while (!conversionComplete);
        return lastResult;
    }
    
    // ========================================
    // Callback-Funktionen
    // ========================================
    
    // Conversion Complete Callback registrieren
    static void attachInterrupt(void (*callback)()) {
        auto irqCallback = getCallback();
        if (irqCallback != nullptr) {
            *irqCallback = callback;
        }
    }
    
    // Interrupt deaktivieren
    static void detachInterrupt() {
        ADC_MODULE::adc->INTCTRL &= ~ADC_RESRDY_bm;
        auto irqCallback = getCallback();
        if (irqCallback != nullptr) {
            *irqCallback = nullptr;
        }
    }
    
    // Wird von ISR aufgerufen
    static void handleInterrupt() {
        // Ergebnis lesen und speichern
        lastResult = ADC_MODULE::adc->RES;
        
        // Flag setzen
        conversionComplete = true;
        
        // Flag löschen
        ADC_MODULE::adc->INTFLAGS = ADC_RESRDY_bm;
        
        // User-Callback aufrufen
        auto callback = getCallback();
        if (callback != nullptr && *callback != nullptr) {
            (*callback)();
        }
    }
    
    // ========================================
    // Erweiterte Funktionen
    // ========================================
    
    // Sample Duration setzen (mehr Samples = höhere Genauigkeit)
    static void setSampleDuration(ADCSampleDuration duration) {
        uint8_t samples = 0;
        switch(duration) {
            case ADCSampleDuration::SAMPLES_2:
                samples = 2;
                break;
            case ADCSampleDuration::SAMPLES_4:
                samples = 4;
                break;
            case ADCSampleDuration::SAMPLES_8:
                samples = 8;
                break;
            case ADCSampleDuration::SAMPLES_16:
                samples = 16;
                break;
            case ADCSampleDuration::SAMPLES_32:
                samples = 32;
                break;
            case ADCSampleDuration::SAMPLES_64:
                samples = 64;
                break;
            case ADCSampleDuration::SAMPLES_128:
                samples = 128;
                break;
            case ADCSampleDuration::SAMPLES_256:
                samples = 256;
                break;
        }
        ADC_MODULE::adc->SAMPCTRL = samples;
    }
    
    // Free Running Mode aktivieren
    static void enableFreeRunning() {
        ADC_MODULE::adc->CTRLA |= ADC_FREERUN_bm;
    }
    
    // Free Running Mode deaktivieren
    static void disableFreeRunning() {
        ADC_MODULE::adc->CTRLA &= ~ADC_FREERUN_bm;
    }
    
    // ========================================
    // Utility-Funktionen
    // ========================================
    
    // ADC-Wert in mV umrechnen (bei VDD Referenz)
    static uint16_t toMillivolts(uint16_t adcValue, uint16_t vddMillivolts = 5000) {
        if (currentResolution == ADCResolution::BITS_8) {
            return (adcValue * vddMillivolts) / 255;
        } else {
            return (adcValue * vddMillivolts) / 1023;
        }
    }
    
    // ADC-Wert in Volt umrechnen (Float)
    static float toVolts(uint16_t adcValue, float vddVolts = 5.0f) {
        if (currentResolution == ADCResolution::BITS_8) {
            return (adcValue * vddVolts) / 255.0f;
        } else {
            return (adcValue * vddVolts) / 1023.0f;
        }
    }
    
    // Internen Temperatursensor lesen (in °C)
    static int16_t readTemperature() {
        // Interne Referenz aktivieren
        uint8_t oldRef = ADC_MODULE::adc->CTRLC;
        ADC_MODULE::adc->CTRLC = (oldRef & ~ADC_REFSEL_gm) | ADC_REFSEL_INTREF_gc;
        
        // Temperatursensor lesen
        uint16_t adcValue = read(ADCChannel::TEMPERATURE);
        
        // Alte Referenz wiederherstellen
        ADC_MODULE::adc->CTRLC = oldRef;
        
        // Formel aus Datenblatt (approximiert)
        // Temp = ((adcValue * 1.1V / 1023) - 0.5V) / 0.01V
        int32_t temp = (((int32_t)adcValue * 1100L) / 1023L - 500L) / 10L;
        return (int16_t)temp;
    }
    
    // VDD Spannung messen (in mV)
    static uint16_t readVDD() {
        // Interne Referenz nutzen (1.1V)
        uint8_t oldRef = ADC_MODULE::adc->CTRLC;
        ADC_MODULE::adc->CTRLC = (oldRef & ~ADC_REFSEL_gm) | ADC_REFSEL_INTREF_gc;
        
        // VDD/10 lesen
        uint16_t adcValue = read(ADCChannel::VDD_DIV10);
        
        // Alte Referenz wiederherstellen
        ADC_MODULE::adc->CTRLC = oldRef;
        
        // VDD = (1.1V * 1023 * 10) / adcValue
        return (uint16_t)((1100UL * 1023UL * 10UL) / adcValue);
    }
};

// ============================================
// Template statische Member-Definitionen
// ============================================

template<typename ADC_MODULE>
bool ADC<ADC_MODULE>::initialized = false;

template<typename ADC_MODULE>
ADCResolution ADC<ADC_MODULE>::currentResolution = ADCResolution::BITS_10;

template<typename ADC_MODULE>
volatile uint16_t ADC<ADC_MODULE>::lastResult = 0;

template<typename ADC_MODULE>
volatile bool ADC<ADC_MODULE>::conversionComplete = false;

#endif //__ADC_H__