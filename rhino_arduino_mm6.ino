/*
 * Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories.
 * Written by Scott Savage, Peter McDermott.
 * Feb 2017-22 GNU General Public License (GPL).
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 *
 * TODO:
 *   Analyze for JPL/etc. C coding rules.
 *   Run through a static analyzer?
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

    if (config.robot_id == CONFIG_ROBOT_ID_NOT_CONFIGURED) {
        if (previous_ok)
            log_writeln(F("ERROR: robot_id == CONFIG_ROBOT_ID_NOT_CONFIGURED. Configure robot and reboot."));
        ok = false;
    }

    ok = (motor_get_thermal_overload_detected() || motor_get_overcurrent_detected()) ? false : ok;

    for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
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
        // sm_state_current = SM_STATE_ERROR;
        log_writeln(F("ERROR: System integrity check failed."));
    }

    static bool button_prev = false;
    bool button = hardware_get_button_pressed();

    if (button_prev != button) {
        log_writeln(F("Button %d"), button);
        button_prev = button;
    }

    sm_execute();

    return;
}
