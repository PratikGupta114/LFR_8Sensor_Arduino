#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <Arduino.h>
#include "PinDefinitions.h"
#include "Config.h"
#include "Defaults.h"
#include "GlobalVariables.h"
#include "MotorControl.h"
#include "Sensor.h"
#include "Wire.h"

#if BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1
#include <ArduinoJson.h>
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#endif

#define TX_DOC_MAX_DATA_LEN 192
#define RX_DOC_MAX_DATA_LEN 64
#define BLUETOOTH_SERIAL Serial

#if BLUETOOTH_LOGGING_ENABLED == 1
StaticJsonDocument<TX_DOC_MAX_DATA_LEN> txDoc;
#endif
#if BLUETOOTH_TUNING_ENABLED == 1
StaticJsonDocument<RX_DOC_MAX_DATA_LEN> rxDoc;
#endif

#define MID_8_SENSORS_HIGH (s3 == 1 && s4 == 1 && s5 == 1 && s6 == 1 && s7 == 1 && s8 == 1 && s9 == 1 && s10 == 1)
#define MID_8_SENSORS_LOW (s3 == 0 && s4 == 0 && s5 == 0 && s6 == 0 && s7 == 0 && s8 == 0 && s9 == 0 && s10 == 0)

#define MID_6_SENSORS_HIGH (s4 == 1 && s5 == 1 && s6 == 1 && s7 == 1 && s8 == 1 && s9 == 1)

// Macro function definitions

int Kp = DEFAULT_KP;
int Ki = DEFAULT_KI;
int Kd = DEFAULT_KD;
int baseMotorSpeed = DEFAULT_MOTOR_SPEED;
int loopDelay = DEFAULT_LOOP_DELAY;
int error = 0;
int leftMotorOffset = 0;
int rightMotorOffset = 0;
int P = 0;
int I = 0;
int D = 0;
int error_dir = 0;
int previousError = 0;
int PID_value = 0;
uint8_t isInverted = BLACK_LINE_WHITE_TRACK;

void indicateOn()
{
	digitalWrite(BUZZER, HIGH);
	digitalWrite(BLUE_LED, HIGH);
}

void indicateOff()
{
	digitalWrite(BUZZER, LOW);
	digitalWrite(BLUE_LED, LOW);
}

void readSensors()
{

	uint16_t sensorData = getSensorReadings();
	error = getCalculatedError(0);

	// left most sensor value
	int s1 = (sensorData & (1 << 13)) >> 13;

	int s2 = (sensorData & (1 << 12)) >> 12;
	int s3 = (sensorData & (1 << 11)) >> 11;
	int s4 = (sensorData & (1 << 10)) >> 10;
	int s5 = (sensorData & (1 << 9)) >> 9;
	int s6 = (sensorData & (1 << 8)) >> 8;
	int s7 = (sensorData & (1 << 7)) >> 7;
	int s8 = (sensorData & (1 << 6)) >> 6;
	int s9 = (sensorData & (1 << 5)) >> 5;
	int s10 = (sensorData & (1 << 4)) >> 4;
	int s11 = (sensorData & (1 << 3)) >> 3;

	// right most sensor value
	int s12 = (sensorData & (1 << 2)) >> 2;

	if (s1 != s12)
		error_dir = s1 - s12;

	if (MID_6_SENSORS_HIGH)
		indicateOn();
	else
		indicateOff();

	// When all the sensors go low
	if (sensorData == 0b0000000000000000)
	{
		// Moved out of the line
		if (error_dir < 0)
			error = OUT_OF_LINE_ERROR_VALUE;
		else if (error_dir > 0)
			error = -1 * OUT_OF_LINE_ERROR_VALUE;
	}
	else if (sensorData == 0b0011111111111100)
	{
		// This is a stop
		moveStraight(baseMotorSpeed, baseMotorSpeed);
		delay(STOP_DELAY);
		uint16_t sensorDataAgain = getSensorReadings();
		if (sensorDataAgain == 0b0011111111111100)
		{
			indicateOff();
			shortBrake(1000);
			stop();
			delay(10000);
		}
	}

	if (isInverted == BLACK_LINE_WHITE_TRACK)
		digitalWrite(WHITE_LED, HIGH);
	else if (isInverted == WHITE_LINE_BLACK_TRACK)
		digitalWrite(WHITE_LED, LOW);

#if BLUETOOTH_LOGGING_ENABLED == 1

	char ss[16];
	char output[TX_DOC_MAX_DATA_LEN];

	sprintf(ss, "%d%d%d%d%d%d%d%d%d%d%d%d", s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12);

	txDoc["di"] = String(ss);
	serializeJson(txDoc, output);
	BLUETOOTH_SERIAL.println(output);

	BLUETOOTH_SERIAL.print("I|Value : ");
	BLUETOOTH_SERIAL.print(String(ss));
	BLUETOOTH_SERIAL.print(" | Error : ");
	BLUETOOTH_SERIAL.print(error);
	BLUETOOTH_SERIAL.print(" i-");
	BLUETOOTH_SERIAL.println(isInverted);
	txDoc.clear();

#endif
}

void calculatePID()
{
	P = error;

	if (error == 0)
		I = 0;
	else
		I = I + error;

	I = constrain(I, -200, 200);

	D = error - previousError;
	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	PID_value = constrain(PID_value, -150, 150);
	previousError = error;
}

void controlMotors()
{
	if (error == OUT_OF_LINE_ERROR_VALUE)
	{
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif
		uint8_t sensorReadings = getSensorReadings();
		while (isOutOfLine(sensorReadings))
		{
			turnCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
			sensorReadings = getSensorReadings();
		}
// #if BRAKING_ENABLED == 1
// 		shortBrake(BRAKE_DURATION_MILLIS);
// #endif
#if GAPS_ENABLED == 1
		error_dir = 0;
#endif
	}
	else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE))
	{
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif
		uint8_t sensorReadings = getSensorReadings();
		while (isOutOfLine(sensorReadings))
		{
			turnCCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
			sensorReadings = getSensorReadings();
		}
// #if BRAKING_ENABLED == 1
// 		shortBrake(BRAKE_DURATION_MILLIS);
// #endif
#if GAPS_ENABLED == 1
		error_dir = 0;
#endif
	}
	else
	{
		int leftMotorSpeed = baseMotorSpeed + PID_value - leftMotorOffset;
		int rightMotorSpeed = baseMotorSpeed - PID_value - rightMotorOffset;

		moveStraight(leftMotorSpeed, rightMotorSpeed);

		if (D != 0)
			delay(loopDelay);
	}
}

void setup()
{

	// Sensor pins data direction set to input
	// DDRA &= ~(1 << S1) & ~(1 << S2) & ~(1 << S3) & ~(1 << S4) & ~(1 << S5);
	// DDRC &= ~(1 << S6) & ~(1 << S7) & ~(1 << S8) & ~(1 << S9) & ~(1 << S10) & ~(1 << S11) & ~(1 << S12);

	// // Pulling up the internal resistors of the pins above
	// PORTA |= (1 << S1) | (1 << S2) | (1 << S3) | (1 << S4) | (1 << S5);
	// PORTC |= (1 << S6) | (1 << S7) | (1 << S8) | (1 << S9) | (1 << S10) | (1 << S11) | (1 << S12);

	// DDRD |= (1 << RIGHT_MOTOR_PIN_1) | (1 << RIGHT_MOTOR_PIN_2);
	// DDRD |= (1 << LEFT_MOTOR_PIN_1) | (1 << LEFT_MOTOR_PIN_1);

	// DDRD |= (1 << RIGHT_MOTOR_PWM_PIN);
	// DDRB |= (1 << LEFT_MOTOR_PWM_PIN);

	// TCCR0 |= (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS00);
	// TCCR2 |= (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS20);

	// DDRA |= (1 << BLUE_LED) | (1 << WHITE_LED) | (1 << BUZZER);

	pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
	pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
	pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
	pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);
	pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
	pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

	pinMode(BLUE_LED, OUTPUT);
	pinMode(WHITE_LED, OUTPUT);
	pinMode(BUZZER, OUTPUT);

	Wire.setClock(400000);
	Wire.begin();

#if BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1
	BLUETOOTH_SERIAL.begin(SERIAL_BAUD_RATE);
#endif
}

void loop()
{

#if BLUETOOTH_TUNING_ENABLED == 1

	if (BLUETOOTH_SERIAL.available())
	{
		BLUETOOTH_SERIAL.flush();
		String data = BLUETOOTH_SERIAL.readStringUntil('\n');
		DeserializationError error = deserializeJson(rxDoc, data);

		if (error)
		{
			// Serial.print("E|deseriaize json failed : ");
			// Serial.println(error.f_str());
			return;
			rxDoc.clear();
		}

		Kp = rxDoc["P"];			  // 0
		Ki = rxDoc["I"];			  // 0
		Kd = rxDoc["D"];			  // 0
		baseMotorSpeed = rxDoc["ms"]; // 255
		loopDelay = rxDoc["de"];	  // 100
		rxDoc.clear();
		// char buff[64];
		// sprintf(buff, "P : %d | I : %d | D : %d | ms : %d | de : %d", P, I, D, ms, de);
		// Serial.println(buff);
	}

#endif
	readSensors();
	calculatePID();
	controlMotors();
}