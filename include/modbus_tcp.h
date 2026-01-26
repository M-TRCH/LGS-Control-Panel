#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>

// =====================================================
// W5500 SPI Pin Configuration for STM32G431CB
// =====================================================
#define W5500_CS_PIN        PB12    // Chip Select สำหรับ W5500

// =====================================================
// Network Configuration
// =====================================================
#define MODBUS_TCP_PORT     502     // Standard Modbus TCP port

// =====================================================
// Coil Configuration
// =====================================================
#define COIL_START_ADDRESS  1001    // Starting coil address (1-based)
#define COIL_COUNT          8       // Number of coils (1001-1008)

// =====================================================
// Function Prototypes
// =====================================================

/**
 * @brief Initialize W5500 Ethernet module
 * @param ip Local IP address
 * @param gateway Gateway IP address
 * @param subnet Subnet mask
 * @return true if initialization successful
 */
bool modbusTcpInit(IPAddress ip, IPAddress gateway, IPAddress subnet);

/**
 * @brief Check if Ethernet link is up
 * @return true if Ethernet link is up
 */
bool modbusTcpIsLinked();

/**
 * @brief Write multiple coils to a Modbus TCP slave
 * @param targetIP IP address of target device
 * @param startAddress Starting coil address (1-based)
 * @param coilValues Array of boolean coil values
 * @param numCoils Number of coils to write
 * @return true if successful
 */
bool modbusTcpWriteCoils(IPAddress targetIP, uint16_t startAddress, bool* coilValues, uint16_t numCoils);

/**
 * @brief Write 8 coils using a byte value (coils 1001-1008)
 * @param targetIP Target IP address
 * @param coilStates 8-bit value representing coil states (bit 0 = coil 1001, etc.)
 * @return true if successful
 */
bool modbusTcpWriteCoilsByte(IPAddress targetIP, uint8_t coilStates);

/**
 * @brief Write single coil
 * @param targetIP Target IP address
 * @param coilAddress Coil address (1-based)
 * @param value true = ON, false = OFF
 * @return true if successful
 */
bool modbusTcpWriteSingleCoil(IPAddress targetIP, uint16_t coilAddress, bool value);

/**
 * @brief Read coils from a Modbus TCP slave
 * @param targetIP IP address of target device
 * @param startAddress Starting coil address (1-based)
 * @param numCoils Number of coils to read
 * @param coilValues Output array for coil values
 * @return true if successful
 */
bool modbusTcpReadCoils(IPAddress targetIP, uint16_t startAddress, uint16_t numCoils, bool* coilValues);

/**
 * @brief Get last Modbus error message
 * @return Error message string
 */
const char* modbusTcpGetLastError();

#endif // MODBUS_TCP_H
