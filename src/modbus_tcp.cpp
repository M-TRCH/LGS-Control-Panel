#include "modbus_tcp.h"
#include "System.h"

// Minimal, focused Modbus/TCP support used by `main.cpp`.

static EthernetClient ethClient;
static ModbusTCPClient modbusClient(ethClient);
static bool isInitialized = false;
static const char* lastError = "No error";

extern HardwareSerial Serial3;

// Quick W5500 presence check (read VERSIONR)
static bool w5500_present()
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(W5500_CS_PIN, LOW);
    SPI.transfer(0x00);
    SPI.transfer(0x39); // VERSIONR
    SPI.transfer(0x00);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(W5500_CS_PIN, HIGH);
    SPI.endTransaction();
    return v == 0x04;
}

bool modbusTcpInit(IPAddress ip, IPAddress gateway, IPAddress subnet)
{
    Serial3.println("[MODBUS] Initializing W5500 and Ethernet...");

    pinMode(W5500_CS_PIN, OUTPUT);
    digitalWrite(W5500_CS_PIN, HIGH);

    // Configure SPI pins (explicit)
    SPI.setMISO(MISO_PIN);
    SPI.setMOSI(MOSI_PIN);
    SPI.setSCLK(SCK_PIN);
    SPI.begin();

    if (!w5500_present())
    {
        Serial3.println("[MODBUS] W5500 not detected (VERSIONR mismatch)");
        lastError = "W5500 not detected";
        isInitialized = false;
        return false;
    }

    Ethernet.init(W5500_CS_PIN);

    byte mac[] = { 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 };
    Ethernet.begin(mac, ip, gateway, gateway, subnet);

    delay(500);

    IPAddress assigned = Ethernet.localIP();
    if (assigned == INADDR_NONE || assigned[0] == 0)
    {
        Serial3.println("[MODBUS] IP assignment failed");
        lastError = "IP assignment failed";
        isInitialized = false;
        return false;
    }

    isInitialized = true;
    lastError = "No error";
    Serial3.print("[MODBUS] Ethernet ready: ");
    Serial3.println(assigned);
    // Set Modbus/TCP response timeout to 1000 ms
    modbusClient.setTimeout(500);
    Serial3.println("[MODBUS] Response timeout set to 1000 ms");
    return true;
}

bool modbusTcpIsLinked()
{
    return Ethernet.linkStatus() == LinkON;
}

bool modbusTcpWriteCoilsByte(IPAddress targetIP, uint8_t coilStates)
{
    if (!isInitialized)
    {
        lastError = "Not initialized";
        return false;
    }

    modbusClient.stop();
    if (!modbusClient.begin(targetIP, MODBUS_TCP_PORT))
    {
        lastError = "Connection failed";
        return false;
    }

    uint16_t modbusAddr = COIL_START_ADDRESS - 1;
    modbusClient.beginTransmission(COILS, modbusAddr, COIL_COUNT);
    for (int i = 0; i < COIL_COUNT; ++i)
    {
        modbusClient.write((coilStates >> i) & 0x01);
    }

    if (!modbusClient.endTransmission())
    {
        lastError = "Write multiple coils failed";
        modbusClient.stop();
        return false;
    }

    modbusClient.stop();
    lastError = "No error";
    return true;
}

bool modbusTcpWriteSingleCoil(IPAddress targetIP, uint16_t unitID, uint16_t coilAddress, bool value)
{
    if (!isInitialized)
    {
        lastError = "Not initialized";
        return false;
    }

    modbusClient.stop();
    if (!modbusClient.begin(targetIP, MODBUS_TCP_PORT))
    {
        lastError = "Connection failed";
        return false;
    }

    // Use coilWrite overload that accepts unit id when available
    if (!modbusClient.coilWrite(unitID, coilAddress, value))
    {
        lastError = "Coil write failed";
        modbusClient.stop();
        return false;
    }

    modbusClient.stop();
    lastError = "No error";
    return true;
}

const char* modbusTcpGetLastError()
{
    return lastError;
}
