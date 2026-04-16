#include "mbed.h"
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include <Eigen/Dense>
#include "LineFollower.h"

#define M_PIf 3.14159265358979323846f // pi
using namespace std::chrono;

// state machine for initialization sequence
enum InitState {
    WAIT_FOR_BUTTON,      // idle, waiting for blue button press
    WAIT_HAND,            // 1 s delay so user can move hand away
    BLIND_DRIVE,          // drive straight, ignore sensors, exit start box
    FIND_INTERSECTION,    // drive straight until outer sensors detect T-intersection
    ALIGN_PIVOT,          // drive forward a bit more to center wheels on intersection
    TURN_LEFT_BLIND,      // pivot left blindly to get off current line
    TURN_LEFT_SEEK,       // continue pivoting left until center sensor finds new line
    LINE_FOLLOW,          // normal PID line following
    CROSS_LINE_WAIT       // stop and wait at cross line (1 s or 3 s)
};

bool do_execute_main_task = false;
bool do_reset_all_once = false;

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

// main runs as an own thread
int main()
{
    thread_sleep_for(500); // wait for serial terminal to open
    printf("--- PES Board System Starting ---\r\n");
    fflush(stdout);

    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20; // 50 Hz main loop
    Timer main_task_timer;

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    DigitalOut led1(PB_7);

    // --- adding variables and objects and applying functions starts here ---

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f;
    const float gear_ratio = 78.125f;
    const float kn = 180.0f / 12.0f;
    // M1 = left motor, M2 = right motor
    DCMotor motor_left(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_right(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // differential drive robot kinematics (values in mm, converted to meters)
    const float d_wheel = 54.56f / 1000.0f; // wheel diameter
    const float b_wheel = 144.0f / 1000.0f; // axel distance
    const float bar_dist = 142.0f / 1000.0f; // distance between motoraxis and led

    // line follower, tune max. vel rps to your needs
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_right.getMaxPhysicalVelocity());

    // --- initialization sequence parameters (tune these on the real robot) ---
    const float drive_vel_rps = 0.5f;          // forward drive speed in rps
    const float turn_vel_rps = 0.5f;           // blind pivot turn speed in rps
    const float turn_seek_vel_rps = 0.25f;    // slower seek speed for accurate line detection
    const float line_follow_vel_rps = 1.0f;    // max wheel velocity during line following
    const int blind_drive_ms = 150;            // time to drive straight out of start box
    const int align_pivot_ms = 100;            // time to drive forward to align wheels on intersection
    const int turn_blind_ms = 400;             // time to pivot blindly off the current line (~30 deg)
    const float outer_sensor_threshold = 0.3f; // threshold for outer sensors detecting intersection
    const float center_sensor_threshold = 0.3f; // threshold for center sensor detecting new line

    // cross line detection by sensor pattern (tune on real robot)
    // 100mm line: outer sensors lit (6+ LEDs active) -> outer > threshold
    // 50mm line:  center sensors lit, outer NOT lit  -> center high, outer low
    const float cross_line_outer_threshold = 0.5f;  // outer sensors above this = 100mm line
    const float cross_line_center_threshold = 0.7f; // center sensors above this = 50mm line
    const int cross_line_wait_50mm_ms = 1000;       // wait time for 50mm cross line
    const int cross_line_wait_100mm_ms = 3000;      // wait time for 100mm cross line
    const int cross_line_cooldown_ms = 500;          // ignore sensors briefly after a stop

    // limit line follower max speed
    lineFollower.setMaxWheelVelocity(line_follow_vel_rps);

    // state machine variables
    InitState init_state = WAIT_FOR_BUTTON;
    Timer state_timer;              // timer for timed state transitions
    Timer cooldown_timer;           // cooldown after cross line stop
    bool cooldown_active = false;   // true = ignore sensors temporarily
    int wait_duration_ms = 0;       // how long to wait at cross line

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true)
    {
        main_task_timer.reset();

        if (do_execute_main_task)
        {
            led1 = 1;
            enable_motors = 1;

            // one-time reset when button is pressed
            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                // start the initialization sequence
                init_state = WAIT_HAND;
                state_timer.reset();
                state_timer.start();
                motor_left.setVelocity(0.0f);
                motor_right.setVelocity(0.0f);
                printf("Button pressed, waiting 1 s...\r\n");
            }

            // --- initialization state machine ---
            switch (init_state)
            {
            case WAIT_FOR_BUTTON:
                // do nothing, waiting for button press
                motor_left.setVelocity(0.0f);
                motor_right.setVelocity(0.0f);
                break;

            case WAIT_HAND:
                // wait 1 second so the user can move their hand
                motor_left.setVelocity(0.0f);
                motor_right.setVelocity(0.0f);
                if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= 1000)
                {
                    init_state = BLIND_DRIVE;
                    state_timer.reset();
                    printf("Blind drive: exiting start box\r\n");
                }
                break;

            case BLIND_DRIVE:
                // drive straight forward, ignore all sensors
                motor_left.setVelocity(drive_vel_rps);
                motor_right.setVelocity(drive_vel_rps);
                if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= blind_drive_ms)
                {
                    init_state = FIND_INTERSECTION;
                    printf("Searching for T-intersection...\r\n");
                }
                break;

            case FIND_INTERSECTION:
            {
                // drive straight until outer sensors detect the horizontal line
                motor_left.setVelocity(drive_vel_rps);
                motor_right.setVelocity(drive_vel_rps);
                float outer = lineFollower.getMeanFourAvgBitsOuter();
                if (outer > outer_sensor_threshold)
                {
                    init_state = ALIGN_PIVOT;
                    state_timer.reset();
                    printf("Intersection found (outer=%.2f), aligning...\r\n", outer);
                }
                break;
            }

            case ALIGN_PIVOT:
                // drive forward a tiny bit more to center wheels on intersection
                motor_left.setVelocity(drive_vel_rps);
                motor_right.setVelocity(drive_vel_rps);
                if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= align_pivot_ms)
                {
                    init_state = TURN_LEFT_BLIND;
                    state_timer.reset();
                    printf("Pivoting left (blind phase)...\r\n");
                }
                break;

            case TURN_LEFT_BLIND:
                // point turn left: left backward, right forward
                motor_left.setVelocity(-turn_vel_rps);
                motor_right.setVelocity(turn_vel_rps);
                if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= turn_blind_ms)
                {
                    init_state = TURN_LEFT_SEEK;
                    printf("Pivoting left (seeking line)...\r\n");
                }
                break;

            case TURN_LEFT_SEEK:
            {
                // continue pivoting left slowly: left backward, right forward
                motor_left.setVelocity(-turn_seek_vel_rps);
                motor_right.setVelocity(turn_seek_vel_rps);
                float center = lineFollower.getMeanFourAvgBitsCenter();
                if (center > center_sensor_threshold)
                {
                    init_state = LINE_FOLLOW;
                    printf("Line found (center=%.2f), switching to line follower\r\n", center);
                }
                break;
            }

            case LINE_FOLLOW:
            {
                // normal PID line following (M1 = left, M2 = right)
                motor_left.setVelocity(lineFollower.getLeftWheelVelocity());
                motor_right.setVelocity(lineFollower.getRightWheelVelocity());

                // expire cooldown if active
                if (cooldown_active &&
                    duration_cast<milliseconds>(cooldown_timer.elapsed_time()).count() >= cross_line_cooldown_ms)
                {
                    cooldown_active = false;
                }

                // detect cross lines by sensor pattern (only when cooldown expired)
                if (!cooldown_active)
                {
                    float outer_val = lineFollower.getMeanFourAvgBitsOuter();
                    float center_val = lineFollower.getMeanFourAvgBitsCenter();

                    if (outer_val > cross_line_outer_threshold)
                    {
                        // 100mm line: outer sensors strongly lit (6+ LEDs)
                        wait_duration_ms = cross_line_wait_100mm_ms;
                        init_state = CROSS_LINE_WAIT;
                        state_timer.reset();
                        printf("100mm line (outer=%.2f, center=%.2f), waiting %d ms\r\n",
                               outer_val, center_val, wait_duration_ms);
                    }
                    else if (center_val > cross_line_center_threshold && outer_val < cross_line_outer_threshold)
                    {
                        // 50mm line: center sensors strongly lit, outer NOT lit
                        wait_duration_ms = cross_line_wait_50mm_ms;
                        init_state = CROSS_LINE_WAIT;
                        state_timer.reset();
                        printf("50mm line (outer=%.2f, center=%.2f), waiting %d ms\r\n",
                               outer_val, center_val, wait_duration_ms);
                    }
                }
                break;
            }

            case CROSS_LINE_WAIT:
                // stop and wait at cross line
                motor_left.setVelocity(0.0f);
                motor_right.setVelocity(0.0f);
                if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= wait_duration_ms)
                {
                    init_state = LINE_FOLLOW;
                    cooldown_active = true;
                    cooldown_timer.reset();
                    cooldown_timer.start();
                    printf("Wait complete, resuming line follow\r\n");
                }
                break;
            }
        }
        else
        {
            // the following code block gets executed only once
            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                enable_motors = 0;
                led1 = 0;
                motor_left.setVelocity(0.0f);
                motor_right.setVelocity(0.0f);
                init_state = WAIT_FOR_BUTTON;
                state_timer.stop();
            }
        }

        // print to the serial terminal
        printf("State: %d | M1: %f | M2: %f\n",
               init_state,
               motor_left.getVelocity(),
               motor_right.getVelocity());

        // toggling the user led
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
 