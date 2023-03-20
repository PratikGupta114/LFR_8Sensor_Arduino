
#include "MotorControl.h"
#include "PinDefinitions.h"
#include <util/delay.h>

#define TURN_SPEED_REDUCTION_FACTOR 20

void moveStraight(int leftMotorSpeed, int rightMotorSpeed)
{
    // Left motor front
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);

    // Right motor front
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
}

void turnCCW(int leftMotorSpeed, int rightMotorSpeed)
{
    // Left motor back
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);

    // Right motor front
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
}

void turnCW(int leftMotorSpeed, int rightMotorSpeed)
{
    // Left motor front
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);

    // Right motor back
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
}

void shortBrake(int durationMillis)
{
    // Left motor front
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);

    // Right motor front
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);

    delay(durationMillis);
}

void stop()
{
    // Left motor stop
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);

    // Right motor stop
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}
