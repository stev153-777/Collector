#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "ColorSensor.h"
#include <cstring>

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main(){

    // set up states for state machine
    enum RobotState {
        INITIAL,
        DRIVING,
        CHECKING_COLOR,
        WAIT_FOR_MAGAZINE,
        REPOSITIONING,
        PICKING,
        PLACING,
        FINISH,
        SLEEP,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

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

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // mechanical button (reference magazine)
    DigitalIn mechanical_button(PC_5);  // create DigitalIn object to evaluate mechanical button, you
                                        // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);     // sets pullup between pin and 3.3 V, so that there
                                        // is a defined potential

    // line follower
    int stopDetected = 0;

    // Pick or Place
    bool picking = false;
    bool placing = false;

    // color sensor
    bool rePosNeeded = false;
    bool color_valid = false;
    int color_num = 0; // define a variable to store the color number, e.g. 0 for red, 1 for green, 2 for blue, 3 for clear
    const char* color_string; // define a variable to store the color string, e.g. "red", "green", "blue", "clear"
    ColorSensor Color_Sensor(PB_3); // create ColorSensor object, connect the frequency output pin of the sensor to PB_3
    int color_retry_counter = 0;
    const int color_retry_delay_cycles = 5; // ~100 ms (5 * 20ms)

    // DC Motor Magazine
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio_M3 = 78.125f; // gear ratio
    const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor magazine_motor(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);
    // enable the motion planner for smooth movement
    magazine_motor.enableMotionPlanner();
    // limit max. velocity to half physical possible velocity
    magazine_motor.setMaxVelocity(magazine_motor.getMaxPhysicalVelocity() * 0.5f);
    float target_rotation   = 0.0f;
    float rotation_red      = 1.0f;
    float rotation_green    = 2.0f;
    float rotation_blue     = 3.0f;
    float rotation_yellow   = 4.0f;
    float positionTolerance = 0.01f;

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    if (enable_motors == 0) enable_motors = 1;

                    // while magazine is not referenced, drive backwards
                    if (mechanical_button.read()) {
                        magazine_motor.setVelocity((-magazine_motor.getMaxVelocity()) * 0.5f);
                    } else {
                        magazine_motor.setVelocity(0.0f);
                        enable_motors = 0;
                        enable_motors = 1;
                        // reset to 0.0f
                        robot_state = RobotState::DRIVING;
                    }
                    break;
                }
                case RobotState::DRIVING: {
                    printf("DRIVING\n");

                    // Drive

                    if(stopDetected){
                        robot_state = RobotState::CHECKING_COLOR;
                    }
                    break;
                }
                
                case RobotState::CHECKING_COLOR: {

                    stopDetected = 0;

                        if (color_retry_counter > 0) {
                            color_retry_counter--;
                            break; // wait before retry
                        }

                        // read the classified color number and store it in the defined variable
                        color_num = Color_Sensor.getColor();

                        // read the classified color string and store it in the defined variable
                        color_string = Color_Sensor.getColorString(color_num);
                        printf("Colour: %s\n", color_string);

                        // move magazine based on colour and apply repositioning
                        switch (color_num) {
                        case 3: // RED
                            rePosNeeded = true;
                            color_valid = true;
                            target_rotation = rotation_red;
                            break;
                        case 5: // GREEN
                            rePosNeeded = true;
                            color_valid = true;
                            target_rotation = rotation_green;
                            break;
                        case 7: // BLUE
                            rePosNeeded = false;
                            color_valid = true;
                            target_rotation = rotation_blue;
                            break;
                        case 4: // YELLOW
                            rePosNeeded = false;
                            color_valid = true;
                            target_rotation = rotation_yellow;
                            break;
                        default:
                            color_valid = false;
                            break;
                        }

                        if(color_valid){
                            magazine_motor.setRotationRelative(target_rotation);    // drive the motor
                            robot_state = RobotState::WAIT_FOR_MAGAZINE;
                        } else {
                            color_retry_counter = color_retry_delay_cycles; // wait before retry
                        }

                    break;
                }
                case RobotState::WAIT_FOR_MAGAZINE: {
                    printf("WAIT_FOR_MAGAZINE\n");

                    // wait until target position is reached
                    if (fabs(magazine_motor.getRotation() - target_rotation) < positionTolerance) {
                        if (rePosNeeded) {
                            robot_state = RobotState::REPOSITIONING;
                        }
                        else {
                            if(picking){
                                placing = false;
                                robot_state = RobotState::PICKING;
                            }
                            else if(placing){
                                picking = false;
                                robot_state = RobotState::PLACING;
                            }
                        }
                    }
                    break;
                }
                case RobotState::REPOSITIONING: {

                    if(picking){
                        placing = false;
                        robot_state = RobotState::PICKING;
                    }
                    else if(placing){
                        picking = false;
                        robot_state = RobotState::PLACING;
                    }
                    break;
                }
                case RobotState::PICKING: {

                    robot_state = RobotState::DRIVING;
                    break;
                }
                case RobotState::PLACING: {

                    robot_state = RobotState::DRIVING;
                    break;
                }
                case RobotState::FINISH: {
                    printf("FINISH\n");

                    break;
                }
                case RobotState::SLEEP: {
                    printf("SLEEP\n");

                    break;
                }
                case RobotState::EMERGENCY: {
                    printf("EMERGENCY\n");

                    break;
                }
                default: {
                    break; // do nothing
                }
            }

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                robot_state = RobotState::INITIAL;
                enable_motors = 0;
                color_valid = false;
                rePosNeeded = false;
                color_retry_counter = 0;
                stopDetected = 0;
                picking = false;
                placing = false;
                target_rotation = 0.0f;
            }
        }

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
