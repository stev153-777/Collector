#include "mbed.h"
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
//#include "FastPWM.h"
#include "DCMotor.h"
#include <Eigen/Dense>
#include "LineFollower.h"

#define M_PIf 3.14159265358979323846f // pi
using namespace std::chrono;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
// decides whether to execute the main task or not
bool do_reset_all_once = false; // this variable is used to reset certain variables and objects and
// shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1); // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
// button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    thread_sleep_for(500); // wait for serial terminal to open
    printf("--- PES Board System Starting ---\r\n");
    fflush(stdout);

    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
    // the main task will run 50 times per second
    Timer main_task_timer; // create Timer object which we use to run the main task
    // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor

    // Change from PB_9 to another pin, e.g., PB_7
    DigitalOut led1(PB_7);

    // --- adding variables and objects and applying functions starts here ---

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    // 6.0f V if you only use one battery pack
    const float gear_ratio = 78.125f;
    const float kn = 180.0f / 12.0f;
    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // differential drive robot kinematics (values in mm, converted to meters)
    const float d_wheel = 54.56f / 1000.0f; // wheel diameter in mm
    const float b_wheel = 147.0f / 1000.0f; // wheelbase, distance from wheel to wheel in mm
    const float bar_dist = 118.0f / 1000.0f; // distance from wheel axis to leds on sensor bar / array in mm

    // line follower, tune max. vel rps to your needs
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true)
    {
        printf("True is true \n");
        printf("MT: %d\n", do_execute_main_task);
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task)
        {
            printf("MT running");
            // --- code that runs when the blue button was pressed goes here ---

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            enable_motors = 1;

            // the following code block gets executed only once
            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects

            }
        }
        else
        {
            // the following code block gets executed only once
            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                enable_motors = 0;
                led1 = 0; // Turn LED off when stopped
                motor_M1.setVelocity(0.0f);
                motor_M2.setVelocity(0.0f);
            }
        }

        // print to the serial terminal
        printf("M1: %f | M2: %f\n",
               motor_M1.getVelocity(),
               motor_M2.getVelocity());

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

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
 