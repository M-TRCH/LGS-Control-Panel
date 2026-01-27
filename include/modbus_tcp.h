#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>

// W5500 configuration
#define W5500_CS_PIN        PB12

// Network and Modbus
#define MODBUS_TCP_PORT     502
#define COIL_START_ADDRESS  1001
#define COIL_COUNT          8

// Public API used by `main.cpp`
bool modbusTcpInit(IPAddress ip, IPAddress gateway, IPAddress subnet);
bool modbusTcpIsLinked();
bool modbusTcpWriteCoilsByte(IPAddress targetIP, uint8_t coilStates);
bool modbusTcpWriteSingleCoil(IPAddress targetIP, uint16_t unitID, uint16_t coilAddress, bool value);
const char* modbusTcpGetLastError();

#endif // MODBUS_TCP_H
