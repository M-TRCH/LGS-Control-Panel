#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
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
static const uint32_t testInterval = 1200;       // Test every 2 seconds
static uint8_t testPattern = 0;
static uint8_t sequenceStep = 0;

// Test patterns
enum TestPattern 
{
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
static void modbusTcpISetup();
static bool modbusTcpLoop();
// Touch-test UI
static void touchTestInit();
static void touchTestLoop();

// Touch-test state
static bool touchBtnState[6] = {false, false, false, false, false, false};
static uint32_t touchLastPress = 0;
static int touchLastActive = -1;

// =====================================================
// TFT and Touchscreen Setup
// =====================================================
// Define pins for TFT and Touchscreen
#define TFT_CS_PIN   PB12
#define TOUCH_CS_PIN PC4

// Create TFT and Touchscreen objects
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen ts(TOUCH_CS_PIN);

void setup()
{
    systemInit();   // Initialize the system

    // -----------------------------------------------------------
    // Ethernet and Modbus TCP Initialization
    // -----------------------------------------------------------
    // Serial3.println("\n========================================");
    // Serial3.println("Modbus TCP Client Test - W5500");
    // Serial3.println("========================================\n");
    // Modbus/W5500 initialization moved to helper
    // modbusTcpISetup();

    // -----------------------------------------------------------
    // TFT and Touchscreen Initialization
    // -----------------------------------------------------------
    SPI.setMISO(MISO_PIN);
    SPI.setMOSI(MOSI_PIN);
    SPI.setSCLK(SCK_PIN);
    SPI.begin();

    // Initialize TFT
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    
    // Initialize touchscreen
    if (!ts.begin())
    {
        Serial3.println("Touchscreen init failed!");
    }
    else
    {
        Serial3.println("Touchscreen started.");
        ts.setRotation(1); // match TFT rotation
    }

    // Draw touch-test UI
    touchTestInit();
}

void loop()
{
    // Touch test UI handling
    touchTestLoop();

    // Blink RUN LED
    static uint32_t lastLedTime = 0;
    static bool ledState = false;
    if (millis() - lastLedTime >= 500)
    {
        lastLedTime = millis();
        ledState = !ledState;
        setLEDBuiltIn(ledState, false, false);
    }
}

// Function to read W5500 register directly
static uint8_t readW5500Register(uint16_t addr)
{
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

// Initialize Modbus/W5500 and print network info
static void modbusTcpISetup()
{
    Serial3.println("[INFO] Initializing W5500...");
    Serial3.print("[INFO] W5500 CS Pin: PB");
    Serial3.println(W5500_CS_PIN == PB12 ? "12" : "6");

    if (modbusTcpInit(localIP, gateway, subnet))
    {
        Serial3.println("[OK] W5500 initialized!");
        Serial3.print("Local IP: ");
        Serial3.println(Ethernet.localIP());
        Serial3.print("Gateway:  ");
        Serial3.println(Ethernet.gatewayIP());
        Serial3.print("Subnet:   ");
        Serial3.println(Ethernet.subnetMask());

        if (Ethernet.linkStatus() == LinkON)
        {
            Serial3.println("[OK] Ethernet Link: UP");
        }
        else
        {
            Serial3.println("[WARN] Ethernet Link: DOWN (check cable)");
        }
    }
    else
    {
        Serial3.println("[ERROR] W5500 init failed!");
        Serial3.println("[ERROR] Check:");
        Serial3.println("  1. SPI wiring (MOSI/MISO/SCK/CS)");
        Serial3.println("  2. W5500 power supply (3.3V)");
        Serial3.println("  3. CS pin not conflicting");
        while (1)
        {
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

// Handle Modbus/TCP periodic work; return false to indicate loop should return early
static bool modbusTcpLoop()
{
    Ethernet.maintain();

    if (!modbusTcpIsLinked())
    {
        setLEDBuiltIn(false, true, false);  // CAL LED = warning

        static uint32_t lastWarnTime = 0;
        if (millis() - lastWarnTime >= 2000)
        {
            lastWarnTime = millis();
            Serial3.println("[WARN] Ethernet Link: DOWN");
        }

        delay(100);
        return false;
    }

    // Run Modbus coil test periodically
    static bool testCoilStateLocal = false;
    static uint16_t testAddressLocal = 1001;
    static uint16_t unitIDLocal = 11;  // Example unit ID

    if (millis() - lastTestTime >= testInterval)
    {
        lastTestTime = millis();

        testCoilStateLocal = !testCoilStateLocal;

        Serial3.print("[TEST] Write single coil - ID: ");
        Serial3.print(unitIDLocal);
        Serial3.print(", Address: ");
        Serial3.print(testAddressLocal);
        Serial3.print(" (");
        Serial3.print(testCoilStateLocal ? "ON" : "OFF");
        Serial3.print(")\t\t\t");
        bool success = modbusTcpWriteSingleCoil(targetIP, unitIDLocal, testAddressLocal, testCoilStateLocal);

        if (testCoilStateLocal == false)
        {
            testAddressLocal++;
            if (testAddressLocal > 1008)
            {
                testAddressLocal = 1001;
            }
        }

        if (success)
        {
            Serial3.println("[OK] Write single coil success");
        }
        else
        {
            Serial3.print("[ERROR] ");
            Serial3.println(modbusTcpGetLastError());
        }
    }

    // Print status every 10 seconds
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime >= 10000)
    {
        lastStatusTime = millis();
        Serial3.print("[STATUS] IP: ");
        Serial3.print(Ethernet.localIP());
        Serial3.print(" | Link: ");
        Serial3.println(Ethernet.linkStatus() == LinkON ? "UP" : "DOWN");
    }

    return true;
}

// =====================================================
// Run Coil Test
// =====================================================
static void runCoilTest()
{
    testCoilState = getTestCoilValue(testPattern, sequenceStep);

    Serial3.print("[TEST] Coils 1001-1008: 0b");
    for (int i = 7; i >= 0; i--)
    {
        Serial3.print((testCoilState >> i) & 1);
    }
    Serial3.print(" (0x");
    if (testCoilState < 0x10)
    {
        Serial3.print("0");
    }
    Serial3.print(testCoilState, HEX);
    Serial3.println(")");

    Serial3.print("[INFO] Connecting to ");
    Serial3.print(targetIP);
    Serial3.println("...");

    // Send Modbus TCP request using ArduinoModbus library
    unsigned long startTime = millis();
    bool success = modbusTcpWriteCoilsByte(targetIP, testCoilState);
    unsigned long elapsed = millis() - startTime;

    if (success)
    {
        Serial3.print("[OK] Coils written successfully (");
        Serial3.print(elapsed);
        Serial3.println(" ms)");
    }
    else
    {
        Serial3.print("[ERROR] ");
        Serial3.print(modbusTcpGetLastError());
        Serial3.print(" (");
        Serial3.print(elapsed);
        Serial3.println(" ms)");
    }

    // Advance pattern
    sequenceStep++;
    if (sequenceStep >= 8)
    {
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

// ---------------------------
// Touch-test UI functions
// ---------------------------
static void drawTouchButton(int idx)
{
    int cols = 3;
    int rows = 2;
    int bw = tft.width() / cols;
    int bh = tft.height() / rows;
    int x = (idx % cols) * bw;
    int y = (idx / cols) * bh;
    int pad = 8;
    uint16_t color = touchBtnState[idx] ? TFT_GREEN : TFT_CYAN;
    tft.fillRect(x + pad, y + pad, bw - 2 * pad, bh - 2 * pad, color);
    tft.drawRect(x + pad, y + pad, bw - 2 * pad, bh - 2 * pad, TFT_WHITE);
    tft.setTextColor(TFT_BLACK, color);
    tft.setTextSize(3);
    String label = String(idx + 1);
    // Center text both horizontally and vertically
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, x + bw / 2, y + bh / 2, 4);
    tft.setTextDatum(TL_DATUM);
}

static void touchTestInit()
{
    tft.fillScreen(TFT_BLACK);
    for (int i = 0; i < 6; i++)
    {
        touchBtnState[i] = false;
        drawTouchButton(i);
    }
}

static int hitTestButton(int mx, int my)
{
    int cols = 3;
    int bw = tft.width() / cols;
    int bh = tft.height() / 2;
    int col = mx / bw;
    int row = my / bh;
    if (col < 0 || col >= 3 || row < 0 || row >= 2) return -1;
    return row * 3 + col;
}

static void touchTestLoop()
{
    // Ensure sampling
    ts.isrWake = true;
    if (ts.touched())
    {
        TS_Point raw = ts.getPoint();
        int mx = map(raw.x, 0, 4095, 0, tft.width() - 1);
        int my = map(raw.y, 0, 4095, 0, tft.height() - 1);
        mx = constrain(mx, 0, tft.width() - 1);
        my = constrain(my, 0, tft.height() - 1);
        int btn = hitTestButton(mx, my);
        if (btn >= 0)
        {
            if (btn != touchLastActive || (millis() - touchLastPress) > 300)
            {
                touchBtnState[btn] = !touchBtnState[btn];
                drawTouchButton(btn);
                touchLastPress = millis();
                touchLastActive = btn;
                Serial3.print("[TOUCH] button "); Serial3.print(btn + 1); Serial3.println(" toggled");
            }
        }
    }
    else
    {
        touchLastActive = -1;
    }
}