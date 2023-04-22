
#include "MotorControl.h"
#include "PinDefinitions.h"
#include "Config.h"
#include "Defaults.h"

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)
void my_delay_ms(int ms)
{
    while (ms > 0)
    {
        _delay_ms(1);
        --ms;
    }
}
#endif

void moveStraight(int leftMotorSpeed, int rightMotorSpeed)
{

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 0)
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
#endif

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)
    // Left motor front
    PORTB |= (1 << LEFT_MOTOR_PIN_1);
    PORTB &= ~(1 << LEFT_MOTOR_PIN_2);

    // Right motor front
    PORTD |= (1 << RIGHT_MOTOR_PIN_1);
    PORTD &= ~(1 << RIGHT_MOTOR_PIN_2);

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    LEFT_MOTOR_PWM_REGISTER = leftMotorSpeed;
    RIGHT_MOTOR_PWM_REGISTER = rightMotorSpeed;
#endif
}

void turnCCW(int leftMotorSpeed, int rightMotorSpeed)
{
#if (BARE_METAL_GPIO_CONTROL_ENABLED == 0)
    // Left motor back
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);

    // Right motor front
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

#if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);

#endif

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)
    // Left motor back
    PORTB &= ~(1 << LEFT_MOTOR_PIN_1);
    PORTB |= (1 << LEFT_MOTOR_PIN_2);

    // Right motor front
    PORTD |= (1 << RIGHT_MOTOR_PIN_1);
    PORTD &= ~(1 << RIGHT_MOTOR_PIN_2);

#if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    LEFT_MOTOR_PWM_REGISTER = leftMotorSpeed;
    RIGHT_MOTOR_PWM_REGISTER = rightMotorSpeed;

#endif
}

void turnCW(int leftMotorSpeed, int rightMotorSpeed)
{
#if (BARE_METAL_GPIO_CONTROL_ENABLED == 0)
    // Left motor front
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);

    // Right motor back
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

#if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
#endif

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)
    // Left motor front
    PORTB |= (1 << LEFT_MOTOR_PIN_1);
    PORTB &= ~(1 << LEFT_MOTOR_PIN_2);

    // Right motor back
    PORTD &= ~(1 << RIGHT_MOTOR_PIN_1);
    PORTD |= (1 << RIGHT_MOTOR_PIN_2);

#if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    LEFT_MOTOR_PWM_REGISTER = leftMotorSpeed;
    RIGHT_MOTOR_PWM_REGISTER = rightMotorSpeed;

#endif
}

void shortBrake(int durationMillis)
{
#if (BARE_METAL_GPIO_CONTROL_ENABLED == 0)
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);

    // Left motor front
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);

    // Right motor front
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    delay(durationMillis);
#endif

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)

    LEFT_MOTOR_PWM_REGISTER = 0;
    RIGHT_MOTOR_PWM_REGISTER = 0;

    // Left motor front
    PORTB |= (1 << LEFT_MOTOR_PIN_1);
    PORTB |= (1 << LEFT_MOTOR_PIN_2);

    // Right motor front
    PORTD |= (1 << RIGHT_MOTOR_PIN_1);
    PORTD |= (1 << RIGHT_MOTOR_PIN_2);

    my_delay_ms(durationMillis);

#endif
}

void stop()
{
#if (BARE_METAL_GPIO_CONTROL_ENABLED == 0)
    // Left motor stop
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);

    // Right motor stop
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
#endif

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)

    // Left motor stop
    PORTB &= ~(1 << LEFT_MOTOR_PIN_1);
    PORTB &= ~(1 << LEFT_MOTOR_PIN_2);

    // Right motor stop
    PORTD &= ~(1 << RIGHT_MOTOR_PIN_1);
    PORTD &= ~(1 << RIGHT_MOTOR_PIN_2);

    LEFT_MOTOR_PWM_REGISTER = 0;
    RIGHT_MOTOR_PWM_REGISTER = 0;

#endif
}
