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
    static bool previous_ok = true;
    bool ok = previous_ok;

    if (previous_ok)
        ok = config_check();

    ok = motor_get_thermal_overload_detected() ? false : ok;

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_state[i].error_flags != 0) {
            if (previous_ok) {
                log_writeln(F(""));
                log_writeln(F("ERROR: motor %c error %d:"), 'A' + i, motor_state[i].error_flags);
                motor_log_errors((motor_id_t)i);
            }
            ok = false;
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
