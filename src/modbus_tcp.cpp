#include "modbus_tcp.h"
#include "System.h"  // For pin definitions

// =====================================================
// Private Variables
// =====================================================
static EthernetClient ethClient;
static ModbusTCPClient modbusTCPClient(ethClient);
static bool isInitialized = false;
static const char* lastError = "No error";

// External Serial for debug (defined in System.cpp)
extern HardwareSerial Serial3;

// =====================================================
// Initialize SPI for W5500
// =====================================================
static void initSPIforW5500() {
    Serial3.println("[DEBUG] Configuring SPI for W5500...");
    Serial3.print("[DEBUG]   MOSI: PA7 (pin ");
    Serial3.print(MOSI_PIN);
    Serial3.println(")");
    Serial3.print("[DEBUG]   MISO: PA6 (pin ");
    Serial3.print(MISO_PIN);
    Serial3.println(")");
    Serial3.print("[DEBUG]   SCK:  PA5 (pin ");
    Serial3.print(SCK_PIN);
    Serial3.println(")");
    Serial3.print("[DEBUG]   CS:   PB12 (pin ");
    Serial3.print(W5500_CS_PIN);
    Serial3.println(")");
    
    // Configure SPI pins explicitly
    SPI.setMISO(MISO_PIN);  // PA6
    SPI.setMOSI(MOSI_PIN);  // PA7
    SPI.setSCLK(SCK_PIN);   // PA5
    
    // Begin SPI with settings for W5500
    // W5500 supports up to 80MHz, Mode 0 or 3
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // Start slow (1MHz)
    SPI.endTransaction();
    
    Serial3.println("[DEBUG] SPI configured (1MHz, Mode 0)");
}

// =====================================================
// W5500 Software Reset via SPI
// =====================================================
static void w5500SoftwareReset() {
    Serial3.println("[DEBUG] Performing W5500 software reset...");
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    
    digitalWrite(W5500_CS_PIN, LOW);
    delayMicroseconds(5);
    
    // W5500 Mode Register (MR) address: 0x0000
    // Write 0x80 to MR to trigger software reset
    SPI.transfer(0x00);  // Address high byte
    SPI.transfer(0x00);  // Address low byte
    SPI.transfer(0x04);  // Control: Write to Common Register
    SPI.transfer(0x80);  // Data: Software Reset
    
    delayMicroseconds(5);
    digitalWrite(W5500_CS_PIN, HIGH);
    
    SPI.endTransaction();
    
    delay(150);  // Wait for reset
    Serial3.println("[DEBUG] Software reset complete");
}

// =====================================================
// Test SPI Communication with W5500
// =====================================================
static bool w5500TestSPI() {
    Serial3.println("[DEBUG] Testing SPI communication...");
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    
    // Test 1: Read Version Register (should be 0x04)
    digitalWrite(W5500_CS_PIN, LOW);
    delayMicroseconds(5);
    
    SPI.transfer(0x00);  // Address high
    SPI.transfer(0x39);  // Address low (VERSIONR)
    SPI.transfer(0x00);  // Control: Read from Common Register
    uint8_t version = SPI.transfer(0x00);  // Read data
    
    delayMicroseconds(5);
    digitalWrite(W5500_CS_PIN, HIGH);
    
    SPI.endTransaction();
    
    Serial3.print("[DEBUG] Version Register: 0x");
    Serial3.println(version, HEX);
    
    if (version == 0x04) {
        Serial3.println("[DEBUG] W5500 detected!");
        return true;
    }
    
    // Additional debug: try reading multiple times
    Serial3.println("[DEBUG] First read failed, trying 3 more times...");
    
    for (int attempt = 0; attempt < 3; attempt++) {
        delay(100);
        
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));  // Try slower
        
        digitalWrite(W5500_CS_PIN, LOW);
        delayMicroseconds(10);
        
        SPI.transfer(0x00);
        SPI.transfer(0x39);
        SPI.transfer(0x00);
        version = SPI.transfer(0x00);
        
        delayMicroseconds(10);
        digitalWrite(W5500_CS_PIN, HIGH);
        
        SPI.endTransaction();
        
        Serial3.print("[DEBUG] Attempt ");
        Serial3.print(attempt + 1);
        Serial3.print(": 0x");
        Serial3.println(version, HEX);
        
        if (version == 0x04) {
            return true;
        }
    }
    
    Serial3.println("[DEBUG] === SPI TROUBLESHOOTING ===");
    Serial3.println("[DEBUG] Version should be 0x04 for W5500");
    Serial3.println("[DEBUG] Got 0x00: No clock/MISO not connected");
    Serial3.println("[DEBUG] Got 0xFF: CS not working/MISO floating");
    Serial3.println("[DEBUG] ");
    Serial3.println("[DEBUG] W5500 Module Pinout:");
    Serial3.println("[DEBUG]   MOSI (or SI)  -> PA7");
    Serial3.println("[DEBUG]   MISO (or SO)  -> PA6");
    Serial3.println("[DEBUG]   SCLK (or SCK) -> PA5");
    Serial3.println("[DEBUG]   SCS (or SS)   -> PB12");
    Serial3.println("[DEBUG]   VCC           -> 3.3V");
    Serial3.println("[DEBUG]   GND           -> GND");
    
    return false;
}

// =====================================================
// Public Functions
// =====================================================

bool modbusTcpInit(IPAddress ip, IPAddress gateway, IPAddress subnet) {
    Serial3.println("[DEBUG] ====================================");
    Serial3.println("[DEBUG] modbusTcpInit() starting...");
    Serial3.println("[DEBUG] ====================================");
    Serial3.flush();
    
    // =====================================================
    // Step 1: Configure CS pin - MUST be HIGH before SPI init
    // =====================================================
    Serial3.print("[DEBUG] Step 1: Setting CS pin PB");
    Serial3.print(W5500_CS_PIN == PB12 ? "12" : (W5500_CS_PIN == PB6 ? "6" : "?"));
    Serial3.println(" as OUTPUT HIGH");
    Serial3.flush();
    
    pinMode(W5500_CS_PIN, OUTPUT);
    digitalWrite(W5500_CS_PIN, HIGH);
    delay(100);
    Serial3.println("[DEBUG] Step 1: DONE");
    Serial3.flush();
    
    // =====================================================
    // Step 2: Initialize SPI (configure pins and settings)
    // =====================================================
    Serial3.println("[DEBUG] Step 2: Calling initSPIforW5500()...");
    Serial3.flush();
    initSPIforW5500();
    delay(100);
    Serial3.println("[DEBUG] Step 2: DONE");
    Serial3.flush();
    
    // =====================================================
    // Step 3: Test SPI communication
    // =====================================================
    Serial3.println("[DEBUG] Step 3: Testing SPI communication...");
    Serial3.flush();
    if (!w5500TestSPI()) {
        Serial3.println("[DEBUG] SPI test failed - trying software reset...");
        Serial3.flush();
        w5500SoftwareReset();
        delay(100);
        
        // Try again after reset
        if (!w5500TestSPI()) {
            Serial3.println("[DEBUG] SPI still not working after reset!");
            Serial3.flush();
            lastError = "SPI communication failed";
            isInitialized = false;
            return false;
        }
    }
    Serial3.println("[DEBUG] Step 3: DONE - SPI OK");
    Serial3.flush();
    
    // =====================================================
    // Step 4: Initialize Ethernet library with CS pin
    // =====================================================
    Serial3.println("[DEBUG] Step 4: Calling Ethernet.init()...");
    Serial3.flush();
    Ethernet.init(W5500_CS_PIN);
    Serial3.println("[DEBUG] Step 4: DONE");
    Serial3.flush();
    
    // =====================================================
    // Step 5: Begin Ethernet with static IP
    // =====================================================
    Serial3.println("[DEBUG] Step 5: Calling Ethernet.begin()...");
    Serial3.print("[DEBUG] IP: ");
    Serial3.println(ip);
    Serial3.flush();
    
    // W5500 requires MAC address (unlike Arduino Opta with built-in MAC)
    // Use locally administered MAC address (bit 1 of first byte = 1)
    byte mac[] = { 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 };
    
    Serial3.print("[DEBUG] MAC: ");
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial3.print("0");
        Serial3.print(mac[i], HEX);
        if (i < 5) Serial3.print(":");
    }
    Serial3.println();
    Serial3.flush();
    
    // Standard W5500 Ethernet.begin with MAC
    Serial3.println("[DEBUG] Calling Ethernet.begin(mac, ip, gateway, gateway, subnet)...");
    Serial3.flush();
    Ethernet.begin(mac, ip, gateway, gateway, subnet);
    Serial3.println("[DEBUG] Step 5: DONE");
    Serial3.flush();
    
    // =====================================================
    // Step 6: Wait for W5500 to stabilize
    // =====================================================
    Serial3.println("[DEBUG] Step 6: Waiting 1000ms for W5500...");
    Serial3.flush();
    delay(1000);
    Serial3.println("[DEBUG] Step 6: DONE");
    Serial3.flush();
    
    // =====================================================
    // Step 7: Check assigned IP
    // =====================================================
    Serial3.println("[DEBUG] Step 7: Checking assigned IP...");
    Serial3.flush();
    IPAddress assignedIP = Ethernet.localIP();
    Serial3.print("[DEBUG] Assigned IP: ");
    Serial3.println(assignedIP);
    Serial3.flush();
    
    if (assignedIP[0] == 0 && assignedIP[1] == 0 && 
        assignedIP[2] == 0 && assignedIP[3] == 0) {
        Serial3.println("[DEBUG] ERROR: IP is 0.0.0.0!");
        Serial3.flush();
        lastError = "IP assignment failed";
        isInitialized = false;
        return false;
    }
    Serial3.println("[DEBUG] Step 7: DONE");
    Serial3.flush();
    
    // =====================================================
    // Step 8: Check hardware link status
    // =====================================================
    Serial3.println("[DEBUG] Step 8: Checking hardware status...");
    Serial3.flush();
    Serial3.print("[DEBUG] Hardware status: ");
    switch (Ethernet.hardwareStatus()) {
        case EthernetNoHardware:
            Serial3.println("No hardware detected!");
            Serial3.flush();
            lastError = "No Ethernet hardware";
            isInitialized = false;
            return false;
        case EthernetW5100:
            Serial3.println("W5100");
            break;
        case EthernetW5200:
            Serial3.println("W5200");
            break;
        case EthernetW5500:
            Serial3.println("W5500");
            break;
        default:
            Serial3.println("Unknown");
            break;
    }
    Serial3.flush();
    
    // Read PHYCFGR directly for debug
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(W5500_CS_PIN, LOW);
    SPI.transfer(0x00);  // Address high
    SPI.transfer(0x2E);  // Address low (PHYCFGR)
    SPI.transfer(0x00);  // Control: Read Common Register
    uint8_t phycfgr = SPI.transfer(0x00);
    digitalWrite(W5500_CS_PIN, HIGH);
    SPI.endTransaction();
    
    Serial3.print("[DEBUG] PHYCFGR: 0x");
    Serial3.print(phycfgr, HEX);
    Serial3.print(" (Bit0=Link: ");
    Serial3.print((phycfgr & 0x01) ? "UP" : "DOWN");
    Serial3.println(")");
    Serial3.flush();
    
    Serial3.print("[DEBUG] Link status (library): ");
    Serial3.println(Ethernet.linkStatus() == LinkON ? "UP" : "DOWN");
    Serial3.flush();
    
    Serial3.println("[DEBUG] Step 8: DONE");
    Serial3.flush();
    
    isInitialized = true;
    lastError = "No error";
    Serial3.println("[DEBUG] ====================================");
    Serial3.println("[DEBUG] modbusTcpInit() SUCCESS!");
    Serial3.println("[DEBUG] ====================================");
    Serial3.flush();
    return true;
}

bool modbusTcpIsLinked() {
    return (Ethernet.linkStatus() == LinkON);
}

bool modbusTcpWriteCoils(IPAddress targetIP, uint16_t startAddress, bool* coilValues, uint16_t numCoils) {
    if (!isInitialized) {
        lastError = "Not initialized";
        return false;
    }
    
    // Always reconnect for each request (more reliable)
    modbusTCPClient.stop();
    
    // Connect to target
    if (!modbusTCPClient.begin(targetIP, MODBUS_TCP_PORT)) {
        lastError = "Connection failed";
        return false;
    }
    
    // Convert 1-based address to 0-based for Modbus library
    uint16_t modbusAddr = startAddress - 1;
    
    // Write each coil individually using coilWrite()
    bool success = true;
    for (uint16_t i = 0; i < numCoils; i++) {
        if (!modbusTCPClient.coilWrite(modbusAddr + i, coilValues[i] ? 0xFF00 : 0x0000)) {
            lastError = "Coil write failed";
            success = false;
            break;
        }
        delay(10);  // Small delay between writes
    }
    
    modbusTCPClient.stop();
    
    if (success) {
        lastError = "No error";
    }
    return success;
}

bool modbusTcpWriteCoilsByte(IPAddress targetIP, uint8_t coilStates) {
    if (!isInitialized) {
        lastError = "Not initialized";
        return false;
    }
    
    // Always reconnect for each request
    modbusTCPClient.stop();

    Serial3.print("[DEBUG] Connecting to ");
    Serial3.print(targetIP);
    Serial3.print(":");
    Serial3.print(MODBUS_TCP_PORT);
    Serial3.println(" ...");

    bool connectResult = modbusTCPClient.begin(targetIP, MODBUS_TCP_PORT);
    Serial3.print("[DEBUG] modbusTCPClient.begin() returned: ");
    Serial3.println(connectResult ? "true" : "false");
    Serial3.print("[DEBUG] ethClient.connected(): ");
    Serial3.println(ethClient.connected() ? "true" : "false");
    Serial3.print("[DEBUG] Local IP: ");
    Serial3.println(Ethernet.localIP());
    Serial3.print("[DEBUG] Local Port: ");
    Serial3.println(ethClient.localPort());
    Serial3.print("[DEBUG] Remote IP: ");
    Serial3.println(ethClient.remoteIP());
    Serial3.print("[DEBUG] Remote Port: ");
    Serial3.println(ethClient.remotePort());

    if (!connectResult) {
        lastError = "Connection failed";
        return false;
    }
    
    // Convert 1-based address to 0-based for Modbus library
    uint16_t modbusAddr = COIL_START_ADDRESS - 1;
    
    // Use beginTransmission for multiple coils (FC 15)
    modbusTCPClient.beginTransmission(COILS, modbusAddr, COIL_COUNT);
    
    // Write each coil value
    for (int i = 0; i < COIL_COUNT; i++) {
        modbusTCPClient.write((coilStates >> i) & 0x01);
    }
    
    // End transmission and send request
    if (!modbusTCPClient.endTransmission()) {
        lastError = "Write multiple coils failed";
        modbusTCPClient.stop();
        return false;
    }
    
    modbusTCPClient.stop();
    lastError = "No error";
    return true;
}

bool modbusTcpWriteSingleCoil(IPAddress targetIP, uint16_t coilAddress, bool value) {
    if (!isInitialized) {
        lastError = "Not initialized";
        return false;
    }
    
    // Always reconnect for each request
    modbusTCPClient.stop();
    
    // Connect to target
    if (!modbusTCPClient.begin(targetIP, MODBUS_TCP_PORT)) {
        lastError = "Connection failed";
        return false;
    }
    
    // Convert 1-based address to 0-based
    uint16_t modbusAddr = coilAddress;
    
    // Write single coil (FC 05) with id (unit id)
    // Value should be 0xFF00 for ON, 0x0000 for OFF
    if (!modbusTCPClient.coilWrite(11, modbusAddr, value)) {
        lastError = "Coil write failed";
        modbusTCPClient.stop();
        return false;
    }
    
    modbusTCPClient.stop();
    lastError = "No error";
    return true;
}

bool modbusTcpReadCoils(IPAddress targetIP, uint16_t startAddress, uint16_t numCoils, bool* coilValues) {
    if (!isInitialized) {
        lastError = "Not initialized";
        return false;
    }
    
    // Always reconnect for each request
    modbusTCPClient.stop();
    
    // Connect to target
    if (!modbusTCPClient.begin(targetIP, MODBUS_TCP_PORT)) {
        lastError = "Connection failed";
        return false;
    }
    
    // Convert 1-based address to 0-based
    uint16_t modbusAddr = startAddress - 1;
    
    // Request coils (FC 01)
    if (!modbusTCPClient.requestFrom(COILS, modbusAddr, numCoils)) {
        lastError = "Read coils failed";
        modbusTCPClient.stop();
        return false;
    }
    
    // Read response
    for (uint16_t i = 0; i < numCoils && modbusTCPClient.available(); i++) {
        coilValues[i] = modbusTCPClient.read() != 0;
    }
    
    modbusTCPClient.stop();
    lastError = "No error";
    return true;
}

const char* modbusTcpGetLastError() {
    return lastError;
}
