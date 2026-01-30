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
// Single-shot Modbus helpers
static bool modbusSingleShotPulse(IPAddress targetIP, uint16_t unitID, uint16_t coilAddress, uint32_t pulseMs);
// Note: no retry wrapper; single write attempt is performed below
// Touch-test UI
static void touchTestInit();
// Returns: index 0..7 when a button is newly latched, -1 otherwise
static int touchTestLoop();

// Touch-test state
static bool touchBtnState[8] = {false, false, false, false, false, false, false, false};
static uint32_t touchLastPress = 0;
static int touchLastActive = -1;
// Button colors (ordered): แดง, เขียว, น้ำเงิน, เหลือง, ฟ้า, ม่วง, ส้ม, ขาว
static const uint16_t touchBtnColors[8] = {TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA, TFT_ORANGE, TFT_WHITE};
// Per-button last press time for debounce
static uint32_t touchBtnLastPress[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Brighten an RGB565 color by a percentage (0-100). Returns RGB565.
static uint16_t brightenColor(uint16_t color, uint8_t percent)
{
    // Extract RGB565 components
    uint8_t r5 = (color >> 11) & 0x1F;
    uint8_t g6 = (color >> 5) & 0x3F;
    uint8_t b5 = color & 0x1F;

    // Convert to 8-bit
    uint8_t r8 = (r5 * 255) / 31;
    uint8_t g8 = (g6 * 255) / 63;
    uint8_t b8 = (b5 * 255) / 31;

    // Brighten towards 255 by percent
    r8 = r8 + ((255 - r8) * percent) / 100;
    g8 = g8 + ((255 - g8) * percent) / 100;
    b8 = b8 + ((255 - b8) * percent) / 100;

    // Convert back to RGB565
    uint16_t nr5 = (r8 * 31 + 127) / 255;
    uint16_t ng6 = (g8 * 63 + 127) / 255;
    uint16_t nb5 = (b8 * 31 + 127) / 255;

    return (uint16_t)((nr5 << 11) | (ng6 << 5) | nb5);
}

// Darken an RGB565 color by a percentage (0-100). Returns RGB565.
static uint16_t darkenColor(uint16_t color, uint8_t percent)
{
    uint8_t r5 = (color >> 11) & 0x1F;
    uint8_t g6 = (color >> 5) & 0x3F;
    uint8_t b5 = color & 0x1F;

    uint8_t r8 = (r5 * 255) / 31;
    uint8_t g8 = (g6 * 255) / 63;
    uint8_t b8 = (b5 * 255) / 31;

    // Darken towards 0 by percent
    r8 = (r8 * (100 - percent)) / 100;
    g8 = (g8 * (100 - percent)) / 100;
    b8 = (b8 * (100 - percent)) / 100;

    uint16_t nr5 = (r8 * 31 + 127) / 255;
    uint16_t ng6 = (g8 * 63 + 127) / 255;
    uint16_t nb5 = (b8 * 31 + 127) / 255;

    return (uint16_t)((nr5 << 11) | (ng6 << 5) | nb5);
}

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
    Serial3.println("\n========================================");
    Serial3.println("Modbus TCP Client Test - W5500");
    Serial3.println("========================================\n");
    modbusTcpISetup();

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
    int pressed = touchTestLoop();
    if (pressed >= 0)
    {
        Serial3.print("[TOUCH] Button "); Serial3.print(pressed + 1); Serial3.println(" action triggered.");
    }

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
    int cols = 4;
    int rows = 2;
    int bw = tft.width() / cols;
    int bh = tft.height() / rows;
    int x = (idx % cols) * bw;
    int y = (idx / cols) * bh;
    int pad = 8;

    // Colors
    uint16_t borderColor = touchBtnColors[idx];
    uint16_t baseFill = touchBtnColors[idx];
    uint16_t effectFill = darkenColor(touchBtnColors[idx], 40); // dim effect when pressed

    int outerW = bw - 2 * pad;
    int outerH = bh - 2 * pad;
    if (outerW <= 0 || outerH <= 0) return;

    // Compute thickness: large but bounded so it fits the button
    int maxThickX = outerW / 2 - 2;
    int maxThickY = outerH / 2 - 2;
    int maxThick = maxThickX < maxThickY ? maxThickX : maxThickY;
    if (maxThick < 1) maxThick = 1;
    const int preferredThick = 14; // very thick
    int thick = preferredThick;
    if (thick > maxThick) thick = maxThick;

    // Draw outer border rectangle (filled with border color)
    // If pressed/latched: dim the entire button area (border + inner)
    if (touchBtnState[idx])
    {
        tft.fillRect(x + pad, y + pad, outerW, outerH, effectFill);
    }
    else
    {
        // Draw outer border rectangle (filled with border color)
        tft.fillRect(x + pad, y + pad, outerW, outerH, borderColor);

        // Draw inner area inset by 'thick' and fill with base color
        int innerX = x + pad + thick;
        int innerY = y + pad + thick;
        int innerW = outerW - 2 * thick;
        int innerH = outerH - 2 * thick;
        if (innerW > 0 && innerH > 0)
        {
            tft.fillRect(innerX, innerY, innerW, innerH, baseFill);
        }
    }
}

static void touchTestInit()
{
    tft.fillScreen(TFT_BLACK);
    for (int i = 0; i < 8; i++)
    {
        touchBtnState[i] = false;
        drawTouchButton(i);
    }
}

static int hitTestButton(int mx, int my)
{
    int cols = 4;
    int bw = tft.width() / cols;
    int bh = tft.height() / 2;
    int col = mx / bw;
    int row = my / bh;
    if (col < 0 || col >= cols || row < 0 || row >= 2) return -1;
    return row * cols + col;
}

static int touchTestLoop()
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
        uint32_t now = millis();
        if (btn >= 0)
        {
            // Per-button debounce (200 ms) to avoid repeated triggers while holding
            if ((int32_t)(now - touchBtnLastPress[btn]) > 200)
            {
                // Toggle behavior: flip state and redraw
                bool newState = !touchBtnState[btn];
                touchBtnState[btn] = newState;
                touchBtnLastPress[btn] = now;
                drawTouchButton(btn);

                // Send persistent Modbus write for the new state
                uint16_t coilAddr = COIL_START_ADDRESS + btn; // map button 0->1001
                uint16_t unitID = 11; // example unit id
                if (modbusTcpWriteSingleCoil(targetIP, unitID, coilAddr, newState))
                {
                    Serial3.print("[MODBUS] Write coil "); Serial3.print(coilAddr);
                    Serial3.print(newState ? " = ON" : " = OFF");
                    Serial3.println(" OK");
                }
                else
                {
                    Serial3.print("[MODBUS] Write coil "); Serial3.print(coilAddr);
                    Serial3.print(newState ? " = ON" : " = OFF");
                    Serial3.print(" FAILED: ");
                    Serial3.println(modbusTcpGetLastError());
                }

                if (newState)
                {
                    Serial3.print("[TOUCH] button "); Serial3.print(btn + 1); Serial3.println(" ON (toggled)");
                    return btn;
                }
                else
                {
                    Serial3.print("[TOUCH] button "); Serial3.print(btn + 1); Serial3.println(" OFF (toggled)");
                }
            }
            touchLastActive = btn;
        }
        else
        {
            // touched but outside buttons -> clear active press marker
            touchLastActive = -1;
        }
    }
    else
    {
        // not touched: clear active press marker so next touch counts
        touchLastActive = -1;
        // reset per-button last press only if needed (not required)
    }

    return -1;
}

// Single-shot Modbus pulse: set coil ON, wait `pulseMs`, then set coil OFF.
// Returns true if both writes succeeded.
static bool modbusSingleShotPulse(IPAddress targetIP, uint16_t unitID, uint16_t coilAddress, uint32_t pulseMs)
{
    Serial3.print("[MODBUS] Single-shot pulse to "); Serial3.print(targetIP);
    Serial3.print(" coil "); Serial3.print(coilAddress);
    Serial3.print(" unit "); Serial3.println(unitID);

    // Set ON
    if (!modbusTcpWriteSingleCoil(targetIP, unitID, coilAddress, true))
    {
        Serial3.print("[ERROR] modbus write ON failed: ");
        Serial3.println(modbusTcpGetLastError());
        return false;
    }

    delay(pulseMs);

    // Set OFF
    if (!modbusTcpWriteSingleCoil(targetIP, unitID, coilAddress, false))
    {
        Serial3.print("[ERROR] modbus write OFF failed: ");
        Serial3.println(modbusTcpGetLastError());
        return false;
    }

    Serial3.println("[MODBUS] Single-shot pulse complete");
    return true;
}

// Retry wrapper for modbus single coil write. Returns true on success.
// Note: retries removed — writes are attempted once using modbusTcpWriteSingleCoil()