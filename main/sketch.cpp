/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "math.h"
#include "stdio.h"

// Pin definitions
#define RIGHT_MOTOR_DIR_REVERSE GPIO_NUM_21
#define RIGHT_MOTOR_DIR_FORWARD GPIO_NUM_23
#define RIGHT_MOTOR_PWM         GPIO_NUM_19

#define LEFT_MOTOR_DIR_REVERSE  GPIO_NUM_25
#define LEFT_MOTOR_DIR_FORWARD  GPIO_NUM_33
#define LEFT_MOTOR_PWM          GPIO_NUM_32

#define MOTOR_DIR_PIN_SEL ((1ULL<<RIGHT_MOTOR_DIR_REVERSE) | (1ULL<<RIGHT_MOTOR_DIR_FORWARD) | (1ULL<<LEFT_MOTOR_DIR_REVERSE) | (1ULL<<LEFT_MOTOR_DIR_FORWARD))

#define DEADBAND 10

// this can be used to correct a bias in the robot direction
// 50 = center, 0 = all the way left, 100 = all the way right
int motorBalance = 50;
bool isBalancePressed = false;

void gpio_init()
{
    Console.printf("Motor control GPIO init\n");
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = MOTOR_DIR_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
}

void pwm_gpio_config()
{
    Console.printf("MCPWM GPIO init\n");
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = RIGHT_MOTOR_PWM,
        .mcpwm0b_out_num = LEFT_MOTOR_PWM,
    };

    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
}

void mcpwm_config()
{
    Console.printf("MCPWM init\n");
    mcpwm_config_t pwm_config;

    // low frequencies work better for driving the motor at low speeds
    pwm_config.frequency = 20;     // frequency = 20Hz
    pwm_config.cmpr_a = 0.0;       // initial duty cycle 0
    pwm_config.cmpr_b = 0.0;       // initial duty cycle 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void motor_control_setup()
{
    gpio_init();
    pwm_gpio_config();
    mcpwm_config();
}

void set_motor_pwm(mcpwm_generator_t motor, float pwm)
{
    // set direction
    if (motor == MCPWM_OPR_A) {
        gpio_set_level(RIGHT_MOTOR_DIR_FORWARD, pwm >= 0);
        gpio_set_level(RIGHT_MOTOR_DIR_REVERSE, pwm < 0);
    }
    if (motor == MCPWM_OPR_B) {
        gpio_set_level(LEFT_MOTOR_DIR_FORWARD, pwm >= 0);
        gpio_set_level(LEFT_MOTOR_DIR_REVERSE, pwm < 0);
    }

    float absolute_pwm = fabs(pwm);
    // TODO: protect motor_x_pwm variables with mutexes
    if (absolute_pwm > 100)
    {
        Console.printf("ERROR: Motor duty cycle cannot exceed 100 \n"); // TODO: make this a proper error log
        absolute_pwm = 0;
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, motor, absolute_pwm);
}

float limit_range(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "vritual device", is a mouse
    // By default it is disabled.
    BP32.enableVirtualDevice(false);

    motor_control_setup();
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];
        float rightMotor = 0.0;
        float leftMotor = 0.0;

        if (myGamepad && myGamepad->isConnected()) {
            /*myGamepad->axisX(),        // (-511 - 512) left X Axis
            myGamepad->axisY(),        // (-511 - 512) left Y axis
            myGamepad->axisRX(),       // (-511 - 512) right X axis
            myGamepad->axisRY(),       // (-511 - 512) right Y axis
            myGamepad->brake(),        // (0 - 1023): brake button
            myGamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button*/

            /*if (myGamepad->throttle() > DEADBAND) {
                rightMotor = 20 + myGamepad->throttle() / 1023.0 * 80;
                leftMotor = 20 + myGamepad->throttle() / 1023.0 * 80;
            }

            if (myGamepad->brake() > DEADBAND) {
                rightMotor = -20 + myGamepad->brake() / 1023.0 * -80;
                leftMotor = -20 + myGamepad->brake() / 1023.0 * -80;
            }*/
            float leftxValue = myGamepad->axisX();
            float leftyValue = myGamepad->axisY();
            float rightxValue = myGamepad->axisRX();
            float rightyValue = myGamepad->axisRY();
            float x = rightxValue;
            float y = rightyValue;
            uint8_t dPad = myGamepad->dpad();
            bool isStartPressed = myGamepad->miscHome();

            if (fabs(leftxValue) > fabs(rightxValue)) {
                x = leftxValue;
            }
            if (fabs(leftyValue) > fabs(rightyValue)) {
                y = leftyValue;
            }

            // make the joystic values x and y exponential such that the max value is still 512 but it's reached slower
            x = x * fabs(x) / 512.0;
            y = y * fabs(y) / 512.0;

            // set max value to 85 since the axis will be added together to get final motor values
            // and the maximum values should be ~100 to avoid clipping
            float maxValue = 85.0;
            float xValue = x / -512.0 * maxValue;
            float yValue = y / -512.0 * maxValue;

            if (fabs(xValue) < DEADBAND && fabs(yValue) < DEADBAND)
            {
                xValue = 0;
                yValue = 0;
            }

            if (dPad == DPAD_UP || dPad == DPAD_UP + DPAD_RIGHT || dPad == DPAD_UP + DPAD_LEFT) {
                yValue = maxValue;
            }
            if (dPad == DPAD_RIGHT || dPad == DPAD_RIGHT + DPAD_UP || dPad == DPAD_RIGHT + DPAD_DOWN) {
                xValue = -maxValue;
            }
            if (dPad == DPAD_DOWN || dPad == DPAD_DOWN + DPAD_RIGHT || dPad == DPAD_DOWN + DPAD_LEFT) {
                yValue = -maxValue;
            }
            if (dPad == DPAD_LEFT || dPad == DPAD_LEFT + DPAD_UP || dPad == DPAD_LEFT + DPAD_RIGHT) {
                xValue = maxValue;
            }

            // Adjust motor balance
            if (dPad == DPAD_LEFT && isStartPressed && !isBalancePressed && motorBalance > 0) {
                motorBalance-=5;
                isBalancePressed = true;
            }
            if (dPad == DPAD_RIGHT && isStartPressed && !isBalancePressed && motorBalance < 100) {
                motorBalance+=5;
                isBalancePressed = true;
            }
            if ((dPad == DPAD_UP || dPad == DPAD_DOWN) && isStartPressed) {
                motorBalance = 50;
            }
            if (dPad != DPAD_RIGHT && dPad != DPAD_LEFT) {
                isBalancePressed = false;
            }

            // These multipliers are values between 0.5 and 1 that compensate for uneven motor power (due to mechanical differences)
            float leftMultiplier = motorBalance < 50 ? 0.5 + (motorBalance / 100.0) : 1.0;
            float rightMultiplier = motorBalance > 50 ? 1.5 - (motorBalance / 100.0) : 1.0;
            leftMotor = limit_range(yValue - xValue, -100, 100) * leftMultiplier;
            rightMotor = limit_range(yValue + xValue, -100, 100) * rightMultiplier;

            // Console.printf("dpad: %d x:%f y:%f max:%f xValue:%f yValue:%f leftMotor:%f rightMotor:%f balance:%d\n", dPad, x, y, maxValue, xValue, yValue, leftMotor, rightMotor, motorBalance);

            set_motor_pwm(MCPWM_OPR_A, rightMotor);
            set_motor_pwm(MCPWM_OPR_B, leftMotor);
        }
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);
    //delay(150);
}
