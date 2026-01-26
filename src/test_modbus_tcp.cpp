/**
 * @file test_modbus_tcp.cpp
 * @brief Test code for Modbus TCP Client with W5500 on STM32G431CB
 * 
 * This test demonstrates:
 * - W5500 Ethernet module initialization via SPI
 * - Modbus TCP Write Multiple Coils (FC 0x0F) to address 1001-1008
 * - Broadcast functionality using Unit ID = 0
 * 
 * Hardware Connection (W5500 to STM32G431CB):
 * - MOSI: PA7 (SPI1)
 * - MISO: PA6 (SPI1)
 * - SCK:  PA5 (SPI1)
 * - CS:   PB6
 * - RST:  PB7 (optional)
 * - VCC:  3.3V
 * - GND:  GND
 * 
 * @note Comment out this file when running normal operation
 */

// Uncomment the line below to enable this test
// #define ENABLE_MODBUS_TCP_TEST

#ifdef ENABLE_MODBUS_TCP_TEST

#include <Arduino.h>
#include <SPI.h>
#include "modbus_tcp.h"
#include "System.h"

// =====================================================
// Network Configuration
// =====================================================
// MAC Address - ต้องเป็น unique ในเครือข่าย
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 };

// IP Configuration - ปรับให้ตรงกับเครือข่ายของคุณ
static IPAddress localIP(192, 168, 1, 100);      // IP ของ STM32
static IPAddress gateway(192, 168, 1, 1);        // Gateway
static IPAddress subnet(255, 255, 255, 0);       // Subnet mask

// Target device IP (หรือใช้ broadcast IP)
static IPAddress targetIP(192, 168, 1, 50);      // IP ของ Modbus Slave
// static IPAddress broadcastIP(192, 168, 1, 255); // Broadcast IP (สำหรับ UDP, ไม่ใช้กับ TCP)

// =====================================================
// Test Variables
// =====================================================
static uint8_t testCoilState = 0x00;    // 8-bit coil states for address 1001-1008
static uint32_t lastTestTime = 0;
static uint32_t testInterval = 2000;    // Test every 2 seconds
static uint8_t testPattern = 0;         // Current test pattern

// =====================================================
// Test Patterns for Coils 1001-1008
// =====================================================
enum TestPattern {
    PATTERN_ALL_OFF = 0,
    PATTERN_ALL_ON,
    PATTERN_ALTERNATE_1,
    PATTERN_ALTERNATE_2,
    PATTERN_SEQUENCE,
    PATTERN_COUNT
};

// =====================================================
// Function Prototypes
// =====================================================
static void printNetworkInfo();
static void runCoilTest();
static uint8_t getTestCoilValue(uint8_t pattern, uint8_t step);

// =====================================================
// Setup Function
// =====================================================
void setup() {
    // Initialize system (LEDs, Serial, etc.)
    systemInit();
    
    Serial3.println("\n========================================");
    Serial3.println("Modbus TCP Client Test - W5500");
    Serial3.println("STM32G431CB + STM32Duino");
    Serial3.println("========================================\n");
    
    // Initialize SPI (shared with encoder, make sure CS pins are managed)
    SPI.begin();
    
    Serial3.println("[INFO] Initializing W5500 Ethernet module...");
    
    // Initialize Modbus TCP
    if (modbusTcpInit(mac, localIP, gateway, subnet)) {
        Serial3.println("[OK] W5500 initialized successfully!");
        printNetworkInfo();
    } else {
        Serial3.println("[ERROR] W5500 initialization failed!");
        Serial3.println("[ERROR] Check cable connection and hardware.");
        
        // Blink error LED
        while (1) {
            setLEDBuiltIn(false, false, true);  // Error LED on
            delay(200);
            setLEDBuiltIn(false, false, false);
            delay(200);
        }
    }
    
    Serial3.println("\n[INFO] Starting Modbus TCP coil test...");
    Serial3.println("[INFO] Writing to coils 1001-1008 every 2 seconds\n");
}

// =====================================================
// Main Loop
// =====================================================
void loop() {
    // Check Ethernet link status
    if (!modbusTcpIsLinked()) {
        Serial3.println("[WARNING] Ethernet link lost!");
        setLEDBuiltIn(false, true, false);  // CAL LED = link warning
        delay(1000);
        return;
    }
    
    // Run coil test periodically
    if (millis() - lastTestTime >= testInterval) {
        lastTestTime = millis();
        runCoilTest();
    }
    
    // Blink RUN LED to show activity
    static uint32_t lastLedTime = 0;
    static bool ledState = false;
    if (millis() - lastLedTime >= 500) {
        lastLedTime = millis();
        ledState = !ledState;
        setLEDBuiltIn(ledState, false, false);
    }
}

// =====================================================
// Print Network Information
// =====================================================
static void printNetworkInfo() {
    Serial3.println("\n--- Network Configuration ---");
    Serial3.print("MAC Address: ");
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial3.print("0");
        Serial3.print(mac[i], HEX);
        if (i < 5) Serial3.print(":");
    }
    Serial3.println();
    
    Serial3.print("Local IP:    ");
    Serial3.println(Ethernet.localIP());
    
    Serial3.print("Gateway:     ");
    Serial3.println(Ethernet.gatewayIP());
    
    Serial3.print("Subnet Mask: ");
    Serial3.println(Ethernet.subnetMask());
    
    Serial3.print("Target IP:   ");
    Serial3.println(targetIP);
    
    Serial3.println("-----------------------------\n");
}

// =====================================================
// Run Coil Test
// =====================================================
static void runCoilTest() {
    static uint8_t sequenceStep = 0;
    
    // Get coil value based on current pattern
    testCoilState = getTestCoilValue(testPattern, sequenceStep);
    
    Serial3.print("[TEST] Pattern: ");
    Serial3.print(testPattern);
    Serial3.print(" | Coils 1001-1008: 0b");
    
    // Print binary representation
    for (int i = 7; i >= 0; i--) {
        Serial3.print((testCoilState >> i) & 1);
    }
    Serial3.print(" (0x");
    if (testCoilState < 0x10) Serial3.print("0");
    Serial3.print(testCoilState, HEX);
    Serial3.println(")");
    
    // Send Modbus TCP request
    ModbusError_t result = modbusTcpBroadcastCoils(targetIP, testCoilState);
    
    if (result == MODBUS_SUCCESS) {
        Serial3.print("[OK] Transaction ID: ");
        Serial3.println(modbusTcpGetLastTransactionId());
    } else {
        Serial3.print("[ERROR] ");
        Serial3.println(modbusTcpGetErrorString(result));
        setLEDBuiltIn(false, false, true);  // Error LED
        delay(100);
        setLEDBuiltIn(false, false, false);
    }
    
    // Advance to next step/pattern
    sequenceStep++;
    if (sequenceStep >= 8) {
        sequenceStep = 0;
        testPattern++;
        if (testPattern >= PATTERN_COUNT) {
            testPattern = 0;
        }
        Serial3.println("\n--- Next Pattern ---\n");
    }
}

// =====================================================
// Get Test Coil Value
// =====================================================
static uint8_t getTestCoilValue(uint8_t pattern, uint8_t step) {
    switch (pattern) {
        case PATTERN_ALL_OFF:
            return 0x00;    // All coils OFF
            
        case PATTERN_ALL_ON:
            return 0xFF;    // All coils ON
            
        case PATTERN_ALTERNATE_1:
            return 0xAA;    // 10101010 - Alternating pattern 1
            
        case PATTERN_ALTERNATE_2:
            return 0x55;    // 01010101 - Alternating pattern 2
            
        case PATTERN_SEQUENCE:
            return (1 << (step % 8));  // Single coil moving: 1, 2, 4, 8, 16, 32, 64, 128
            
        default:
            return 0x00;
    }
}

// =====================================================
// Manual Coil Control Functions (for interactive testing)
// =====================================================

/**
 * @brief Set individual coil state
 * @param coilNumber Coil number (1-8, corresponding to address 1001-1008)
 * @param state true = ON, false = OFF
 */
void setCoil(uint8_t coilNumber, bool state) {
    if (coilNumber < 1 || coilNumber > 8) {
        Serial3.println("[ERROR] Invalid coil number (1-8)");
        return;
    }
    
    if (state) {
        testCoilState |= (1 << (coilNumber - 1));
    } else {
        testCoilState &= ~(1 << (coilNumber - 1));
    }
    
    Serial3.print("[INFO] Coil ");
    Serial3.print(1000 + coilNumber);
    Serial3.print(" = ");
    Serial3.println(state ? "ON" : "OFF");
    
    modbusTcpBroadcastCoils(targetIP, testCoilState);
}

/**
 * @brief Set all coils at once
 * @param states 8-bit value (bit 0 = coil 1001, bit 7 = coil 1008)
 */
void setAllCoils(uint8_t states) {
    testCoilState = states;
    
    Serial3.print("[INFO] All coils set to: 0x");
    Serial3.println(testCoilState, HEX);
    
    modbusTcpBroadcastCoils(targetIP, testCoilState);
}

#endif // ENABLE_MODBUS_TCP_TEST
