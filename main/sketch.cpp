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
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using, "Serial" you can use Bluepad32 "Console" class instead.
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

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Console.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
        // clang-format on
    };
    static const char* modifier_names[] = {
        // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
        // clang-format on
    };
    Console.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = key_names[key-4];
            Console.printf("%s,", keyName);
       }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = modifier_names[key-0xe0];
            Console.printf("%s,", keyName);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Console.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);

    // See ArduinoController.h for all the available functions.
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {

    if (!ctl->isAnyKeyPressed())
        return;

    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Console.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Console.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Console.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    dumpBalanceBoard(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

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

    motor_control_setup();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

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
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);
    //delay(150);
}
