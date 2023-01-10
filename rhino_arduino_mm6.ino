/*
 * Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories.
 * Written by Scott Savage, Peter McDermott.
 * Copyright (C) 2017-2022 by the authors. Licensed under the GNU General Public License (GPL).
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "config.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"
#include "sm.h"

void setup()
{
    // https://www.arduino.cc/reference/en/language/structure/sketch/setup/.

    sm_init();
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
        unsigned char error_flags = motor_get_and_clear_error_flags(i);

        if (error_flags != prev_error_flags[i]) {
            for (int j = 0; j < MOTOR_ERROR_FLAG_COUNT; j++) {
                int error_bit = 1 << j;
                if ((error_flags & error_bit) != (prev_error_flags[i] & error_bit)) {
                    log_writeln();
                    log_write(error_flags & error_bit ? F("*ERROR TRIGGERED*") : F("*ERROR CLEARED*"));
                    log_write(F(" Motor %c: "), 'A' + i);
                    motor_log_error_flags(error_bit);
                    log_writeln();
                }

                if ((error_flags & error_bit) == MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED)
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

    static bool previous_check_system_integrity_ok = true;

    if (previous_check_system_integrity_ok && !check_system_integrity()) {
        previous_check_system_integrity_ok = false;
        sm_set_next_state(sm_state_error_enter);
        log_writeln(F("ERROR: System integrity check failed. Reboot or power-cycle system."));
    }

    sm_execute();

    return;
}
