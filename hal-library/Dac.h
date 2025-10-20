/*
 * DAC.h
 *
 * Created: 19.10.2025 22:00:00
 * Author: WART3K
 */

#ifndef __DAC_H__
#define __DAC_H__

#include <avr/io.h>

#include <stdint.h>

// ============================================
// DAC-Konfiguration Enumerationen
// ============================================

enum class DACReference {
    VDD,            // VDD als Referenz
    INTERNAL_1V1,   // Interne 1.1V Referenz
    INTERNAL_2V5,   // Interne 2.5V Referenz
    INTERNAL_4V3    // Interne 4.3V Referenz
};

// ============================================
// DAC-Wrapper-Struct
// ============================================

struct Dac0 {
    static constexpr volatile DAC_t* dac = &DAC0;
    static constexpr uint8_t dacIndex = 0;
    // DAC Output: PD6
};

// ============================================
// DAC Template-Klasse
// ============================================

template<typename DAC_MODULE>
class DAC {
private:
    static bool initialized;
    static DACReference currentReference;
    
public:
    // ========================================
    // Basis DAC-Funktionen
    // ========================================
    
    // DAC initialisieren
    static void begin(DACReference reference = DACReference::VDD) {
        currentReference = reference;
        
        // TODO: korrekte Referenzspannung setzen
        switch(reference) {
            case DACReference::INTERNAL_1V1:
                VREF.CTRLA |= VREF_DAC0REFSEL_1V5_gc;
                DAC_MODULE::dac->CTRLA = DAC_REFSEL_INTREF_gc;
                break;
            case DACReference::INTERNAL_2V5:
                VREF.CTRLA |= VREF_DAC0REFSEL_2V5_gc;
                DAC_MODULE::dac->CTRLA = DAC_REFSEL_INTREF_gc;
                break;
            case DACReference::INTERNAL_4V3:
                VREF.CTRLA |= VREF_DAC0REFSEL_4V34_gc;
                DAC_MODULE::dac->CTRLA = DAC_REFSEL_INTREF_gc;
                break;
            case DACReference::VDD:
            default:
                DAC_MODULE::dac->CTRLA = DAC_REFSEL_VDDREF_gc;
                break;
        }
        
        // DAC Output aktivieren
        DAC_MODULE::dac->CTRLA |= DAC_OUTEN_bm;
        
        // DAC aktivieren
        DAC_MODULE::dac->CTRLA |= DAC_ENABLE_bm;
        
        initialized = true;
    }
    
    // DAC deaktivieren
    static void end() {
        DAC_MODULE::dac->CTRLA &= ~DAC_ENABLE_bm;
        DAC_MODULE::dac->CTRLA &= ~DAC_OUTEN_bm;
        initialized = false;
    }
    
    // ========================================
    // Output-Funktionen
    // ========================================
    
    // 8-Bit Wert ausgeben (0-255)
    static void write(uint8_t value) {
        if (!initialized) return;
        DAC_MODULE::dac->DATA = value;
    }
    
    // Prozent ausgeben (0.0 - 100.0)
    static void writePercent(float percent) {
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        uint8_t value = (uint8_t)(percent * 2.55f);
        write(value);
    }
    
    // Spannung ausgeben (in mV)
    static void writeMillivolts(uint16_t millivolts) {
        uint16_t refVoltage = getReferenceMillivolts();
        if (millivolts > refVoltage) millivolts = refVoltage;
        uint8_t value = (uint8_t)((millivolts * 255UL) / refVoltage);
        write(value);
    }
    
    // Spannung ausgeben (in Volt, Float)
    static void writeVolts(float volts) {
        writeMillivolts((uint16_t)(volts * 1000.0f));
    }
    
    // ========================================
    // Utility-Funktionen
    // ========================================
    
    // Aktuelle Referenzspannung in mV
    static uint16_t getReferenceMillivolts() {
        switch(currentReference) {
            case DACReference::INTERNAL_1V1:
                return 1100;
            case DACReference::INTERNAL_2V5:
                return 2500;
            case DACReference::INTERNAL_4V3:
                return 4300;
            case DACReference::VDD:
            default:
                return 5000;  // Annahme: 5V VDD
        }
    }
};

// ============================================
// Template statische Member-Definitionen
// ============================================

template<typename DAC_MODULE>
bool DAC<DAC_MODULE>::initialized = false;

template<typename DAC_MODULE>
DACReference DAC<DAC_MODULE>::currentReference = DACReference::VDD;

#endif //__DAC_H__