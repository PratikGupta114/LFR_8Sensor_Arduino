
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "GlobalVariables.h"
#include "PinDefinitions.h"
#include "Config.h"
#include "Wire.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

#define TWO_BYTES_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define TWO_BYTES_TO_BINARY(twoBytes)    \
    (twoBytes & 0x8000 ? '1' : '0'),     \
        (twoBytes & 0x4000 ? '1' : '0'), \
        (twoBytes & 0x2000 ? '1' : '0'), \
        (twoBytes & 0x1000 ? '1' : '0'), \
        (twoBytes & 0x0800 ? '1' : '0'), \
        (twoBytes & 0x0400 ? '1' : '0'), \
        (twoBytes & 0x0200 ? '1' : '0'), \
        (twoBytes & 0x0100 ? '1' : '0'), \
        (twoBytes & 0x0080 ? '1' : '0'), \
        (twoBytes & 0x0040 ? '1' : '0'), \
        (twoBytes & 0x0020 ? '1' : '0'), \
        (twoBytes & 0x0010 ? '1' : '0'), \
        (twoBytes & 0x0008 ? '1' : '0'), \
        (twoBytes & 0x0004 ? '1' : '0'), \
        (twoBytes & 0x0002 ? '1' : '0'), \
        (twoBytes & 0x0001 ? '1' : '0')

#define TWELVE_BIT_SENSOR_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c"
#define TO_TWELVE_BIT_SENSOR_PATTERN(twoBytes) \
    (twoBytes & 0x2000 ? '1' : '0'),           \
        (twoBytes & 0x1000 ? '1' : '0'),       \
        (twoBytes & 0x0800 ? '1' : '0'),       \
        (twoBytes & 0x0400 ? '1' : '0'),       \
        (twoBytes & 0x0200 ? '1' : '0'),       \
        (twoBytes & 0x0100 ? '1' : '0'),       \
        (twoBytes & 0x0080 ? '1' : '0'),       \
        (twoBytes & 0x0040 ? '1' : '0'),       \
        (twoBytes & 0x0020 ? '1' : '0'),       \
        (twoBytes & 0x0010 ? '1' : '0'),       \
        (twoBytes & 0x0008 ? '1' : '0'),       \
        (twoBytes & 0x0004 ? '1' : '0')

uint16_t getSensorData(uint8_t leftHalfPinState, uint8_t rightHalfPinState)
{
    // Shift the leftHalfPinStates towards left only by 6 bits
    uint16_t processedSensorData = 0x0000;
    processedSensorData |= ((uint16_t)leftHalfPinState << 6);
    // Set the first two bits of leftHalfPinStates LSB to 0 in processedSensorData.
    processedSensorData &= ~(1 << 6) & ~(1 << 7);
    processedSensorData |= rightHalfPinState;
    // Set the first two bits of rightHalfPinStates LSB to 0 in processedSensorData.
    processedSensorData &= ~(1 << 0) & ~(1 << 1);

    return processedSensorData;
}

// returns a 2-byte data where each bit represent the state of each sensor
uint16_t getSensorReadings()
{
    uint16_t reading = 0x0000;

    uint8_t leftHalfPinState;
    uint8_t rightHalfPinState;

    Wire.requestFrom(LEFT_HALF_ADDRESS, 1);
    if (Wire.available() >= 1)
        leftHalfPinState = Wire.read();

    Wire.requestFrom(RIGHT_HALF_ADDRESS, 1);
    if (Wire.available() >= 1)
        rightHalfPinState = Wire.read();

    reading = getSensorData(leftHalfPinState, rightHalfPinState);

    // #if BLUETOOTH_LOGGING_ENABLED == 1
    //     char buffer[100];
    //     snprintf(buffer, 100, "I|Sensor Pattern : " TWO_BYTES_TO_BINARY_PATTERN, TWO_BYTES_TO_BINARY(reading));
    //     Serial.println(buffer);
    //     memset(buffer, 0x00, 50);
    // #endif

    // invert the sensor readings
    if (isInverted == BLACK_LINE_WHITE_TRACK)
    {
        reading ^= 0b0011111111111100;
    }

    return reading;
}

// the following function calculates the error value in the following format

int getCalculatedError(int fallbackError)
{
    uint16_t sensorReading = getSensorReadings();
    int numeratorSum = 0, denominatorSum = 0;

    // Assuming that the the MSB represents the index 0 of the array (left to right)
    for (int i = 2; i <= (SENSOR_COUNT + 1); i++)
    {
        // Check the digital values at the ith bit
        // int sensorValue = (((sensorReading & (1 << (SENSOR_COUNT - 1 - i))) >> (SENSOR_COUNT - 1 - i)) > 0) ? 1 : 0;
        uint16_t sensorValue = ((sensorReading & (1 << (15 - i))) >> (15 - i));
        numeratorSum += (i - 1) * 100 * (int)sensorValue;
        denominatorSum += sensorValue;
    }

    int error = fallbackError;
    if (denominatorSum != 0)
        error = ((numeratorSum / (denominatorSum * 50)) - (SENSOR_COUNT + 1));
    return error;
    // return (((numeratorSum * 2) / denominatorSum) - ((SENSOR_COUNT + 1) * 1000)) / 1000;
}

int isOutOfLine(uint16_t sensorReadings)
{
    uint16_t options[] = {
        0b0000011000000000,
        0b0000011100000000,
        // 0b0000011110000000,
        0b0000001000000000,
        0b0000001100000000,
        0b0000001110000000,
        // 0b0000001111000000,
        0b0000000100000000,
        0b0000000110000000,
        0b0000000111000000,
        // 0b0000000111100000,
        0b0000000010000000,
        0b0000000011000000,
        0b0000000011100000,
        // 0b0000000011110000,
        0b0000000001000000,
        0b0000000001100000,
        0b0011100111111100,
        0b0011100011111100,
        // 0b0011100001111100,
        0b0011110111111100,
        0b0011110011111100,
        0b0011110001111100,
        // 0b0011110000111100,
        0b0011111011111100,
        0b0011111001111100,
        0b0011111000111100,
        // 0b0011111000011100,
        0b0011111101111100,
        0b0011111100111100,
        0b0011111100011100,
        // 0b0011111100001100,
        0b0011111110111100,
        0b0011111110011100,
    };

    int n = sizeof(options) / sizeof(uint16_t);

    for (int i = 0; i < n; i++)
    {
        if (sensorReadings == options[i])
            return 0;
    }

    return 1;
}
