#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "FastPWM.h"
// #include "LineFollower.h" // Commented out for straight drive

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // --- adding variables and objects and applying functions starts here ---

    // 1. Motor setup
    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs (12V)
    const float gear_ratio  = 78.125f; // gear ratio 78.125:1
    const float kn          = 180.0f / 12.0f; // motor speed constant [rpm/V]

    // motor M1 (right) and M2 (left)
    FastPWM motor_M1(PB_PWM_M1);
    FastPWM motor_M2(PB_PWM_M2);

    // 2. Line Follower setup (Commented out for straight drive)
    /*
    const float d_wheel  = 0.0372f; // wheel diameter in meters
    const float b_wheel  = 0.1560f; // wheelbase in meters
    const float bar_dist = 0.1140f; // distance from wheel axis to sensor bar in meters
    const float max_vel_rps = motor_M1.getMaxPhysicalVelocity(); // max speed in rotations per second
    LineFollower line_follower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, max_vel_rps);
    */

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            // Enable motors
            enable_motors = 1;

            // --- STRAIGHT DRIVE ---
            // Set a fixed speed for both motors in rotations per second (rps)
            // 0.4f is a slow, controlled speed to start with
            float speed_rps = 0.4f; 

            motor_M1.write(0.25f); // apply -6V to the motor
            motor_M2.write(0.25f); // apply -6V to the motor

            /*
            motor_M1.setVelocity(speed_rps);
            motor_M2.setVelocity(speed_rps);
            */
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // Stop and disable motors
                enable_motors = 0;

                /*
                motor_M1.setVelocity(0.0f);
                motor_M2.setVelocity(0.0f);
                */

            }
        }

        // toggling the user led for heartbeat
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
 