// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

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
int motor_balance = 50;
bool is_balance_pressed = false;
int64_t last_boost_started = 0;
float x_value = 0;
float y_value = 0;
float max_value = 100.0;
bool is_boosting = false;
bool shoulder_button_previous_status = false;

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
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    // Stop when a controller disconnects
    x_value = 0;
    y_value = 0;
    set_motor_pwm(MCPWM_OPR_A, 0.0);
    set_motor_pwm(MCPWM_OPR_B, 0.0);

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks, and the default behavior for scanning or not.
    // By default, if the "startScanning" parameter is not passed, it will do the "start scanning".
    // Notice that "Start scanning" will try to auto-connect to devices that are compatible with Bluepad32.
    // E.g: if a Gamepad, keyboard or mouse are detected, it will try to auto connect to them.
    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Notice that scanning can be stopped / started at any time by calling:
    // BP32.enableNewBluetoothConnections(enabled);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    motor_control_setup();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    float right_motor = 0.0;
    float left_motor = 0.0;

    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    
    if (dataUpdated) {
        for (auto myController : myControllers) {
            if (!myController || !myController->isConnected()) {
                continue;
            }
            if (!myController->isGamepad()) {
                Console.printf("Unsupported controller\n");
                continue;
            }
            
            float left_x_value = myController->axisX();
            // Ignore left Y to make it more clear it's only used for steering
            // float left_y_value = myController->axisY();
            float right_x_value = myController->axisRX();
            float right_y_value = myController->axisRY();
            float x = right_x_value;
            float y = right_y_value;
            uint8_t d_pad = myController->dpad();
            bool is_start_pressed = myController->miscHome();

            if (fabs(left_x_value) > fabs(right_x_value)) {
                x = left_x_value;
            }
            /* Ignore left Y to make it more clear it's only used for steering
            if (fabs(left_y_value) > fabs(right_y_value)) {
                y = left_y_value;
            } */

            // make the joystic values x and y exponential such that the max value is still 512 but it's reached slower
            x = x * fabs(x) / 512.0;
            y = y * fabs(y) / 512.0;
            x_value = x / -512.0 * max_value;
            y_value = y / -512.0 * max_value;

            // Override Y values if A or B button is pressed (Accelerate / Reverse)
            if (myController->a()) {
                y_value = max_value;
            }
            if (myController->b()) {
                y_value = -max_value;
            }

            if (fabs(x_value) < DEADBAND && fabs(y_value) < DEADBAND)
            {
                x_value = 0;
                y_value = 0;
            }

            if (d_pad == DPAD_UP || d_pad == DPAD_UP + DPAD_RIGHT || d_pad == DPAD_UP + DPAD_LEFT) {
                y_value = max_value;
            }
            if (d_pad == DPAD_RIGHT || d_pad == DPAD_RIGHT + DPAD_UP || d_pad == DPAD_RIGHT + DPAD_DOWN) {
                x_value = -max_value;
            }
            if (d_pad == DPAD_DOWN || d_pad == DPAD_DOWN + DPAD_RIGHT || d_pad == DPAD_DOWN + DPAD_LEFT) {
                y_value = -max_value;
            }
            if (d_pad == DPAD_LEFT || d_pad == DPAD_LEFT + DPAD_UP || d_pad == DPAD_LEFT + DPAD_RIGHT) {
                x_value = max_value;
            }

            // Adjust motor balance
            if (d_pad == DPAD_LEFT && is_start_pressed && !is_balance_pressed && motor_balance > 0) {
                motor_balance-=5;
                is_balance_pressed = true;
            }
            if (d_pad == DPAD_RIGHT && is_start_pressed && !is_balance_pressed && motor_balance < 100) {
                motor_balance+=5;
                is_balance_pressed = true;
            }
            if ((d_pad == DPAD_UP || d_pad == DPAD_DOWN) && is_start_pressed) {
                motor_balance = 50;
            }
            if (d_pad != DPAD_RIGHT && d_pad != DPAD_LEFT) {
                is_balance_pressed = false;
            }

            int64_t time_since_last_boost = (esp_timer_get_time() / 1000) - last_boost_started;
            bool is_shoulder_button_down = myController->l1() || myController->r1();
            // Boost
            if (time_since_last_boost > 5000 && y_value > 0.1 && is_shoulder_button_down && !shoulder_button_previous_status) {
                // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
                // It is possible to set it by calling:
                // Some controllers have two motors: "strong motor", "weak motor".
                // It is possible to control them independently.
                last_boost_started = esp_timer_get_time() / 1000;
                time_since_last_boost = 0;
            }
            if (time_since_last_boost < 1000 && is_shoulder_button_down) {
                myController->playDualRumble(0 /* delayedStartMs */, 10 /* durationMs */, 64 /* weakMagnitude */, 32 /* strongMagnitude */);
                // Set the LED color to red when boosting
                myController->setColorLED(255, 0, 0);
                is_boosting = true;
            } else {
                is_boosting = false;
                // Set the LED color blue during cooldown and turn purple when the boost is ready
                myController->setColorLED(time_since_last_boost > 5000 ? 255 : 0, 0, 255);
            }

            shoulder_button_previous_status = is_shoulder_button_down;
        }
    }

    // These multipliers are values between 0.5 and 1 that compensate for uneven motor power (due to mechanical differences)
    float left_multiplier = motor_balance < 50 ? 0.5 + (motor_balance / 100.0) : 1.0;
    float right_multiplier = motor_balance > 50 ? 1.5 - (motor_balance / 100.0) : 1.0;

    int max_pwm = 45;
    float y_value_with_overrides = y_value;
    float x_value_with_overrides = x_value;
    if (is_boosting) {
        max_pwm = 100;
        // When boost is on, we always go forwards with full steam!
        y_value_with_overrides = max_value;
        // ...and steering does not work quite as well
        x_value_with_overrides = x_value * 0.75;
    }

    left_motor = limit_range(y_value_with_overrides - x_value_with_overrides, -max_pwm, max_pwm) * left_multiplier;
    right_motor = limit_range(y_value_with_overrides + x_value_with_overrides, -max_pwm, max_pwm) * right_multiplier;

    // Console.printf("dpad: %d x:%f y:%f max:%f x_value:%f y_value:%f left_motor:%f right_motor:%f balance:%d\n", d_pad, x, y, max_value, x_value, y_value, left_motor, right_motor, motor_balance);

    // Console.printf("left:%f right:%f\n", left_motor, right_motor);

    set_motor_pwm(MCPWM_OPR_A, right_motor);
    set_motor_pwm(MCPWM_OPR_B, left_motor);

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);
    //delay(150);
}
