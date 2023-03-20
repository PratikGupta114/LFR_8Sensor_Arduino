#pragma once

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

uint16_t getSensorReadings();
int getCalculatedError(int fallbackError);
int isOutOfLine(uint16_t sensorReadings);

#endif