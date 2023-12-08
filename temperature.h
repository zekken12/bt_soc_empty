#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <stdint.h>
#include "sl_sensor_rht.h"

// Function to convert temperature to BLE format
int16_t convertTemperatureToBLE(float temperatureCelsius);

// Function to get and convert temperature to BLE format
sl_status_t getAndConvertTemperatureToBLE(int16_t *bleTemperature);

#endif /* TEMPERATURE_H_ */
