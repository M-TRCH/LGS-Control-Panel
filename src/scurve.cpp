#include "scurve.h"

#if !defined(MODBUS_TCP_TEST_MODE) || (MODBUS_TCP_TEST_MODE == false)

ScurveProfile scurve;
unsigned long start_scurve_time = 0;

#endif // !MODBUS_TCP_TEST_MODE