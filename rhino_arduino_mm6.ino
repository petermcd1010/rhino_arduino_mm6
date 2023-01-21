/*
 * Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories.
 * Written by Scott Savage, Peter McDermott.
 * Copyright (C) 2017-2022 by the authors. Licensed under the GNU General Public License (GPL).
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

/*
 * You must install the ros_lib 'Rhinoserial Arduino Library' in the Arduino
 * environment to be able to control these robots from your computer using the
 * Robot Operating System (ROS) framework. To install ros_lib in the Arduino
 * IDE select the 'Tools' menu, 'Manage Libraries', and then scroll down to
 * find and install the 'Rosserial Arduino Library by Michael Ferguson.'
 *
 * On MacOS, there is an issue with rosserial and cstring (see
 * https://github.com/ros-drivers/rosserial/issues/518). You may need to edit
 * ~/Documents/Arduino/libraries/Rosserial_Arduino_Library/src/ros/msg.h to
 * make ros compile.
 */
#include <ros.h>  // Note:

#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#include "config.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"
#include "sm.h"

// https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros

#define BUTTON 8
#define LED 13

ros::NodeHandle node_handle;

std_msgs::String button_msg;
std_msgs::UInt16 led_msg;

void subscriberCallback(const std_msgs::UInt16& led_msg)
{
    if (led_msg.data == 1)
        digitalWrite(LED, HIGH);
    else
        digitalWrite(LED, LOW);
}

ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber < std_msgs::UInt16 > led_subscriber("toggle_led", &subscriberCallback);


void setup()
{
    // https://www.arduino.cc/reference/en/language/structure/sketch/setup/.

    sm_init();

    node_handle.initNode();
    node_handle.advertise(button_publisher);
    node_handle.subscribe(led_subscriber);
}

static bool check_system_integrity()
{
    // Return false if config_check() fails or a thermal overload is triggered.

    static unsigned char prev_error_flags[MOTOR_ID_COUNT] = {};

    static bool previous_ok = true;
    bool ok = previous_ok;

    if (previous_ok)
        ok = config_check();

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        unsigned char error_flags = motor_get_error_flags(i);

        if (error_flags != prev_error_flags[i]) {
            for (int j = 0; j < MOTOR_ERROR_COUNT; j++) {
                int error_bit = 1 << j;
                if ((error_flags & error_bit) != (prev_error_flags[i] & error_bit)) {
                    log_writeln();
                    log_write(error_flags & error_bit ? F("*ERROR TRIGGERED*") : F("*ERROR CLEARED*"));
                    log_write(F(" Motor %c: "), 'A' + i);
                    log_write((const __FlashStringHelper *)motor_error_name_by_id[j]);
                    log_writeln(F("."));
                }

                if ((error_flags & error_bit) == (1 << MOTOR_ERROR_THERMAL_OVERLOAD_DETECTED))
                    ok = false;
            }
            prev_error_flags[i] = error_flags;
        }
    }

    previous_ok = ok;
    return ok;
}

void loop()
{
    // https://www.arduino.cc/reference/en/language/structure/sketch/loop/.

    static bool prev_ok = true;
    bool ok = check_system_integrity();

    if (prev_ok && !ok) {
        prev_ok = false;
        sm_set_next_state(sm_state_error_enter);
        log_writeln(F("ERROR: System error detected. Reboot or power-cycle system."));
    }

    sm_execute();

    if (digitalRead(BUTTON) == HIGH)
        button_msg.data = "Pressed";
    else
        button_msg.data = "NOT pressed";

    button_publisher.publish(&button_msg);
    node_handle.spinOnce();

    return;
}
