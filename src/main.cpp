#include <Arduino.h>
#include <SPI.h>
#include "system.h"
#include "modbus_tcp.h"

// =====================================================
// Network Configuration
// =====================================================
static IPAddress localIP(192, 168, 0, 199);      // IP ของ STM32
static IPAddress gateway(192, 168, 0, 1);        // Gateway
static IPAddress subnet(255, 255, 255, 0);       // Subnet mask
static IPAddress targetIP(192, 168, 0, 101);      // IP ของ Modbus Slave

// =====================================================
// Test Variables
// =====================================================
static uint8_t testCoilState = 0x00;
static uint32_t lastTestTime = 0;
static const uint32_t testInterval = 3000;       // Test every 2 seconds
static uint8_t testPattern = 0;
static uint8_t sequenceStep = 0;

// Test patterns
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
static uint8_t getTestCoilValue(uint8_t pattern, uint8_t step);
static void runCoilTest();

void setup() 
{
    systemInit();   // Initialize the system
    
    Serial3.println("\n========================================");
    Serial3.println("Modbus TCP Client Test - W5500");
    Serial3.println("========================================\n");
    
    // Note: SPI is initialized inside modbusTcpInit()
    Serial3.println("[INFO] Initializing W5500...");
    Serial3.print("[INFO] W5500 CS Pin: PB");
    Serial3.println(W5500_CS_PIN == PB12 ? "12" : "6");
    
    // Initialize Modbus TCP (no MAC needed, like working code)
    if (modbusTcpInit(localIP, gateway, subnet)) {
        Serial3.println("[OK] W5500 initialized!");
        Serial3.print("Local IP: ");
        Serial3.println(Ethernet.localIP());
        Serial3.print("Gateway:  ");
        Serial3.println(Ethernet.gatewayIP());
        Serial3.print("Subnet:   ");
        Serial3.println(Ethernet.subnetMask());
        
        // Check link status
        if (Ethernet.linkStatus() == LinkON) {
            Serial3.println("[OK] Ethernet Link: UP");
        } else {
            Serial3.println("[WARN] Ethernet Link: DOWN (check cable)");
        }
    } else {
        Serial3.println("[ERROR] W5500 init failed!");
        Serial3.println("[ERROR] Check:");
        Serial3.println("  1. SPI wiring (MOSI/MISO/SCK/CS)");
        Serial3.println("  2. W5500 power supply (3.3V)");
        Serial3.println("  3. CS pin not conflicting");
        while (1) {
            setLEDBuiltIn(false, false, true);
            delay(200);
            setLEDBuiltIn(false, false, false);
            delay(200);
        }
    }
    
    Serial3.println("\n[INFO] Modbus TCP Client Ready!");
    Serial3.print("[INFO] Target: ");
    Serial3.print(targetIP);
    Serial3.println(":502");
    Serial3.println("[INFO] Coils: 1001-1008 (FC15)");
    Serial3.println("[INFO] Test interval: 2 seconds\n");
}

// Function to read W5500 register directly
static uint8_t readW5500Register(uint16_t addr) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(W5500_CS_PIN, LOW);
    SPI.transfer((addr >> 8) & 0xFF);  // Address high
    SPI.transfer(addr & 0xFF);          // Address low
    SPI.transfer(0x00);                 // Control: Read Common Register
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(W5500_CS_PIN, HIGH);
    SPI.endTransaction();
    return val;
}

void loop()
{
    // IMPORTANT: Must call maintain() for W5500
    Ethernet.maintain();
    
    // Check Ethernet link
    if (!modbusTcpIsLinked()) {
        setLEDBuiltIn(false, true, false);  // CAL LED = warning
        
        static uint32_t lastWarnTime = 0;
        if (millis() - lastWarnTime >= 2000) {
            lastWarnTime = millis();
            Serial3.println("[WARN] Ethernet Link: DOWN");
        }
        delay(100);
        return;
    }
    
    
    // Run Modbus coil test periodically
    static bool testCoilState = true;
    if (millis() - lastTestTime >= testInterval) {
        lastTestTime = millis();
        // ทดสอบ write single coil id=11 (address=11)
        testCoilState = !testCoilState;
        Serial3.print("[TEST] Write single coil id=11 (");
        Serial3.print(testCoilState ? "ON" : "OFF");
        Serial3.println(")");
        bool success = modbusTcpWriteSingleCoil(targetIP, 1001, testCoilState);
        
        if (success) {
            Serial3.println("[OK] Write single coil id=11 success");
        } else {
            Serial3.print("[ERROR] ");
            Serial3.println(modbusTcpGetLastError());
        }
    }
    
    // Print status every 10 seconds
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime >= 10000) {
        lastStatusTime = millis();
        Serial3.print("[STATUS] IP: ");
        Serial3.print(Ethernet.localIP());
        Serial3.print(" | Link: ");
        Serial3.println(Ethernet.linkStatus() == LinkON ? "UP" : "DOWN");
    }
    
    // Blink RUN LED
    static uint32_t lastLedTime = 0;
    static bool ledState = false;
    if (millis() - lastLedTime >= 500) {
        lastLedTime = millis();
        ledState = !ledState;
        setLEDBuiltIn(ledState, false, false);
    }
}

// =====================================================
// Run Coil Test
// =====================================================
static void runCoilTest() {
    testCoilState = getTestCoilValue(testPattern, sequenceStep);
    
    Serial3.print("[TEST] Coils 1001-1008: 0b");
    for (int i = 7; i >= 0; i--) {
        Serial3.print((testCoilState >> i) & 1);
    }
    Serial3.print(" (0x");
    if (testCoilState < 0x10) Serial3.print("0");
    Serial3.print(testCoilState, HEX);
    Serial3.println(")");
    
    Serial3.print("[INFO] Connecting to ");
    Serial3.print(targetIP);
    Serial3.println("...");
    
    // Send Modbus TCP request using ArduinoModbus library
    unsigned long startTime = millis();
    bool success = modbusTcpWriteCoilsByte(targetIP, testCoilState);
    unsigned long elapsed = millis() - startTime;
    
    if (success) {
        Serial3.print("[OK] Coils written successfully (");
        Serial3.print(elapsed);
        Serial3.println(" ms)");
    } else {
        Serial3.print("[ERROR] ");
        Serial3.print(modbusTcpGetLastError());
        Serial3.print(" (");
        Serial3.print(elapsed);
        Serial3.println(" ms)");
    }
    
    // Advance pattern
    sequenceStep++;
    if (sequenceStep >= 8) {
        sequenceStep = 0;
        testPattern = (testPattern + 1) % PATTERN_COUNT;
        Serial3.println("--- Next Pattern ---\n");
    }
}

// =====================================================
// Get Test Coil Value
// =====================================================
static uint8_t getTestCoilValue(uint8_t pattern, uint8_t step) {
    switch (pattern) {
        case PATTERN_ALL_OFF:     return 0x00;
        case PATTERN_ALL_ON:      return 0xFF;
        case PATTERN_ALTERNATE_1: return 0xAA;
        case PATTERN_ALTERNATE_2: return 0x55;
        case PATTERN_SEQUENCE:    return (1 << (step % 8));
        default:                  return 0x00;
    }
}