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
#include <util/delay.h>

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

enum class RS485Mode {
    DISABLED,   // Standard UART/RS-232
    ENABLED     // RS-485 mit Direction Control
};

// ============================================
// USART-Wrapper-Structs mit RS-485 Support
// ============================================

struct Usart0 {
    static constexpr volatile USART_t* usart = &USART0;
    static constexpr uint8_t usartIndex = 0;
    
    // RS-485 Direction Enable Pin (DE/RE)
    static constexpr volatile PORT_t* dePort = &PORTA;
    static constexpr uint8_t dePin = 2;  // PA2 als DE/RE
    
    static inline void configurePins() {
        // TX=PA0 als Output, RX=PA1 als Input (mit Pull-up)
        PORTA.DIRSET = PIN0_bm;     // TX als Output
        PORTA.DIRCLR = PIN1_bm;     // RX als Input
        PORTA.PIN1CTRL = PORT_PULLUPEN_bm;  // Pull-up für RX
    }
    
    static inline void configureRS485() {
        // DE/RE Pin als Output
        dePort->DIRSET = (1 << dePin);
        // Initial: RX Mode (LOW)
        dePort->OUTCLR = (1 << dePin);
    }
    
    static inline void enableTransmit() {
        // DE/RE HIGH = Transmit Mode
        dePort->OUTSET = (1 << dePin);
        // Kleine Verzögerung für Transceiver
        _delay_us(1);
    }
    
    static inline void enableReceive() {
        // DE/RE LOW = Receive Mode
        dePort->OUTCLR = (1 << dePin);
    }
};

struct Usart1 {
    static constexpr volatile USART_t* usart = &USART1;
    static constexpr uint8_t usartIndex = 1;
    
    // RS-485 Direction Enable Pin (DE/RE)
    static constexpr volatile PORT_t* dePort = &PORTC;
    static constexpr uint8_t dePin = 2;  // PC2 als DE/RE
    
    static inline void configurePins() {
        // TX=PC0 als Output, RX=PC1 als Input
        PORTC.DIRSET = PIN0_bm;
        PORTC.DIRCLR = PIN1_bm;
        PORTC.PIN1CTRL = PORT_PULLUPEN_bm;
    }
    
    static inline void configureRS485() {
        dePort->DIRSET = (1 << dePin);
        dePort->OUTCLR = (1 << dePin);
    }
    
    static inline void enableTransmit() {
        dePort->OUTSET = (1 << dePin);
        _delay_us(1);
    }
    
    static inline void enableReceive() {
        dePort->OUTCLR = (1 << dePin);
    }
};

struct Usart2 {
    static constexpr volatile USART_t* usart = &USART2;
    static constexpr uint8_t usartIndex = 2;
    
    // RS-485 Direction Enable Pin (DE/RE)
    static constexpr volatile PORT_t* dePort = &PORTF;
    static constexpr uint8_t dePin = 2;  // PF2 als DE/RE
    
    static inline void configurePins() {
        // TX=PF0 als Output, RX=PF1 als Input
        PORTF.DIRSET = PIN0_bm;
        PORTF.DIRCLR = PIN1_bm;
        PORTF.PIN1CTRL = PORT_PULLUPEN_bm;
    }
    
    static inline void configureRS485() {
        dePort->DIRSET = (1 << dePin);
        dePort->OUTCLR = (1 << dePin);
    }
    
    static inline void enableTransmit() {
        dePort->OUTSET = (1 << dePin);
        _delay_us(1);
    }
    
    static inline void enableReceive() {
        dePort->OUTCLR = (1 << dePin);
    }
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
// UART Template-Klasse
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
    
    // RS-485 Mode Flag
    static bool rs485Enabled;
    
    // Baud-Rate berechnen
    static inline uint16_t calculateBaud(uint32_t baudrate, uint32_t cpuFreq) {
        // Formel: BAUD = (64 * f_CLK_PER) / (S * f_BAUD)
        // S = 16 für normale Geschwindigkeit
        return (uint16_t)((cpuFreq * 64UL) / (16UL * baudrate));
    }
    
    // Callback-Array ermitteln
    static inline void (**getRxCallback())() {
        switch (USART::usartIndex) {
            case 0: return &UartCallbacks::rxCallback0;
            case 1: return &UartCallbacks::rxCallback1;
            case 2: return &UartCallbacks::rxCallback2;
            default: return nullptr;
        }
    }
    
public:
    // ========================================
    // Basis UART-Funktionen
    // ========================================
    
    // UART initialisieren (mit optionalem RS-485 Mode)
    static void begin(uint32_t baudrate,
                      UartDataBits dataBits = UartDataBits::BITS_8,
                      UartParity parity = UartParity::NONE,
                      UartStopBits stopBits = UartStopBits::ONE,
                      RS485Mode rs485 = RS485Mode::DISABLED) {
        
        // Pins konfigurieren
        USART::configurePins();
        
        // RS-485 aktivieren?
        rs485Enabled = (rs485 == RS485Mode::ENABLED);
        if (rs485Enabled) {
            USART::configureRS485();
            USART::enableReceive();  // Initial: Receive Mode
        }
        
        // Baud-Rate setzen
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
    
    // UART beenden
    static void end() {
        // TX und RX deaktivieren
        USART::usart->CTRLB &= ~(USART_TXEN_bm | USART_RXEN_bm);
        
        // Interrupts deaktivieren
        USART::usart->CTRLA &= ~(USART_RXCIE_bm | USART_TXCIE_bm);
        
        // Buffer leeren
        rxHead = 0;
        rxTail = 0;
        txHead = 0;
        txTail = 0;
        
        // RS-485 deaktivieren
        rs485Enabled = false;
    }
    
    // ========================================
    // Sende-Funktionen (mit RS-485 Support)
    // ========================================
    
    // Einzelnes Byte senden (RS-485 aware)
    static void write(uint8_t data) {
        // RS-485: Auf TX umschalten
        if (rs485Enabled) {
            USART::enableTransmit();
        }
        
        // Warten bis Transmit-Buffer leer ist
        while (!(USART::usart->STATUS & USART_DREIF_bm));
        
        // Daten senden
        USART::usart->TXDATAL = data;
        
        // Warten bis Byte komplett gesendet (wichtig für RS-485!)
        while (!(USART::usart->STATUS & USART_TXCIF_bm));
        USART::usart->STATUS = USART_TXCIF_bm;  // Flag löschen
        
        // RS-485: Zurück auf RX umschalten
        if (rs485Enabled) {
            _delay_us(1);  // Kurze Verzögerung für Transceiver
            USART::enableReceive();
        }
    }
    
    // Array senden (optimiert für RS-485)
    static void write(const uint8_t* data, uint16_t length) {
        if (length == 0) return;
        
        // RS-485: Einmal auf TX umschalten
        if (rs485Enabled) {
            USART::enableTransmit();
        }
        
        // Alle Bytes senden
        for (uint16_t i = 0; i < length; i++) {
            while (!(USART::usart->STATUS & USART_DREIF_bm));
            USART::usart->TXDATAL = data[i];
        }
        
        // Warten bis letztes Byte komplett gesendet
        while (!(USART::usart->STATUS & USART_TXCIF_bm));
        USART::usart->STATUS = USART_TXCIF_bm;
        
        // RS-485: Zurück auf RX
        if (rs485Enabled) {
            _delay_us(1);
            USART::enableReceive();
        }
    }
    
    // ========================================
    // Empfangs-Funktionen
    // ========================================
    
    // Bytes verfügbar?
    static uint8_t available() {
        return (RX_BUFFER_SIZE + rxHead - rxTail) % RX_BUFFER_SIZE;
    }
    
    // Einzelnes Byte lesen
    static uint8_t read() {
        // Warten bis Daten verfügbar
        if (rxHead == rxTail) {
            return 0;
        }
        
        uint8_t data = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
        return data;
    }
    
    // Buffer leeren
    static void flush() {
        rxHead = 0;
        rxTail = 0;
    }
    
    // ========================================
    // Interrupt-Funktionen
    // ========================================
    
    // RX Interrupt Callback registrieren
    static void attachRxInterrupt(void (*callback)()) {
        auto rxCallback = getRxCallback();
        if (rxCallback != nullptr) {
            *rxCallback = callback;
        }
    }
    
    // RX Interrupt deaktivieren
    static void detachRxInterrupt() {
        auto rxCallback = getRxCallback();
        if (rxCallback != nullptr) {
            *rxCallback = nullptr;
        }
    }
    
    // Wird von ISR aufgerufen
    static void handleRxInterrupt() {
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
    
    static void handleTxInterrupt() {
        // TODO: Implementierung für buffered TX
    }
    
    // ========================================
    // RS-485 spezifische Funktionen
    // ========================================
    
    // Prüfen ob RS-485 Modus aktiv ist
    static bool isRS485Enabled() {
        return rs485Enabled;
    }
    
    // Manuell TX aktivieren (für spezielle Anwendungsfälle)
    static void setTransmitMode() {
        if (rs485Enabled) {
            USART::enableTransmit();
        }
    }
    
    // Manuell RX aktivieren (für spezielle Anwendungsfälle)
    static void setReceiveMode() {
        if (rs485Enabled) {
            USART::enableReceive();
        }
    }
};

// ============================================
// Template statische Member-Definitionen
// ============================================

template<typename USART>
volatile uint8_t UART<USART>::rxBuffer[UART<USART>::RX_BUFFER_SIZE];

template<typename USART>
volatile uint8_t UART<USART>::rxHead = 0;

template<typename USART>
volatile uint8_t UART<USART>::rxTail = 0;

template<typename USART>
volatile uint8_t UART<USART>::txBuffer[UART<USART>::TX_BUFFER_SIZE];

template<typename USART>
volatile uint8_t UART<USART>::txHead = 0;

template<typename USART>
volatile uint8_t UART<USART>::txTail = 0;

template<typename USART>
bool UART<USART>::rs485Enabled = false;

#endif //__USART_H__