// This is the main State Machine Program

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "ColorSensor.h"
#include <cstring>
#include "UltrasonicSensor.h"
#include <cmath>
#include <Eigen/Dense>
#include "LineFollower.h"

#define M_PIf 3.14159265358979323846f // pi
using namespace std::chrono;

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
        BLIND_DRIVE,          // drive straight, ignore sensors, exit start box
        FIND_INTERSECTION,    // drive straight until outer sensors detect T-intersection
        ALIGN_PIVOT,          // drive forward a bit more to center wheels on intersection
        TURN_LEFT_BLIND,      // pivot left blindly to get off current line
        TURN_LEFT_SEEK,       // continue pivoting left until center sensor finds new line
        LINE_FOLLOW,          // normal PID line following
        CROSS_LINE_STOP,
        CHECKING_COLOR,
        WAIT_FOR_MAGAZINE_INIT,
        REPOSITIONING,
        ARM_DOWN,
        WAIT_ARM_DOWN,
        ARM_UP,
        WAIT_ARM_UP,
        CHECK_PACKAGE,
        WAIT_FOR_MAGAZINE,
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
    // DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // mechanical button (reference magazine)
    DigitalIn mechanical_button(PC_5);  // create DigitalIn object to evaluate mechanical button, you
                                        // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);     // sets pullup between pin and 3.3 V, so that there
                                        // is a defined potential


    // mechanical button (skip Drive)
    DigitalIn skip_drive(PB_1);
    skip_drive.mode(PullUp);
    // mechanical button (read color and pick)
    DigitalIn readcolor(PB_2);
    readcolor.mode(PullUp);

    // line follower
    // int stopDetected = 0;

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

    // state machine variables
    // InitState init_state = WAIT_FOR_BUTTON;
    Timer state_timer;              // timer for timed state transitions
    Timer cooldown_timer;           // cooldown after cross line stop
    bool cooldown_active = false;   // true = ignore sensors temporarily
    int wait_duration_ms = 0;       // how long to wait at cross line

    // limit line follower max speed
    lineFollower.setMaxWheelVelocity(line_follow_vel_rps);

    // ultrasonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;
    bool success = false;

    // Pick or Place
    int packages_picked = 0;
    int packages_placed = 0;
    bool picking = false;
    bool placing = false;

    // color sensor
    bool rePosNeeded = false;
    bool color_valid = false;
    int color_num = 0; // define a variable to store the color number, e.g. 0 for red, 1 for green, 2 for blue, 3 for clear
    const char* color_string; // define a variable to store the color string, e.g. "red", "green", "blue", "clear"
    ColorSensor Color_Sensor(PB_3); // create ColorSensor object, connect the frequency output pin of the sensor to PB_3
    int color_retry_counter = 0;
    const int color_retry_delay_cycles = 25; // 500 ms
    // const int color_retry_delay_cycles = 5; // ~100 ms (5 * 20ms)
    const int max_color_retries = 5;
    int color_attempts = 0;
    float color_raw_Hz[4] = {0.0f};
    float color_avg_Hz[4] = {0.0f};
    float color_cal[4] = {0.0f};        

    // DC Motor Magazine
    const float voltage_max_mag = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio_M3 = 390.625f; // gear ratio
    const float kn_M3 = 36.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor magazine_motor(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max_mag);
    // enable the motion planner for smooth movement
    magazine_motor.enableMotionPlanner();
    // limit max. velocity to half physical possible velocity
    // magazine_motor.setMaxVelocity(magazine_motor.getMaxPhysicalVelocity() * 0.5f);
    float target_position_absolute = 0.0f;
    // float reference_zero    = 0.0f;
    float velocity_100 = magazine_motor.getMaxVelocity();
    float velocity_10 = (magazine_motor.getMaxVelocity()*0.1);
    float velocity_20 = (magazine_motor.getMaxVelocity()*0.2);
    float target_rotation   = 0.0f;
    float rotation_red      = 0.17f;
    float rotation_green    = 0.27f; // 0.5f
    float rotation_blue     = 0.67f; // 0.75
    float rotation_yellow   = 0.77f;
    float positionTolerance = 0.0005f;
    float grip_offset       = 0.2f;
    float color_active      = 0.0f;
    int i                   = 0;
    // float curr_pos          = 0.0f;
    // bool executePositioning = false;
    // bool referenced         = false;
    // bool moving             = false;
    // bool armDown            = false;
    // bool armUp              = false;
    magazine_motor.setVelocity(0.0f);
    magazine_motor.setMaxVelocity(velocity_100);

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // start timer
    main_task_timer.start();
    // Start Timer
    state_timer.reset();
    state_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---
            us_distance_cm = us_sensor.read();

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("%f ", magazine_motor.getRotation());
                    printf("INITIAL\n");
                    
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    if (enable_motors == 0) enable_motors = 1;

                    // while magazine is not referenced, drive forwards until reference button
                    if (!mechanical_button.read()) {
                        magazine_motor.setVelocity(velocity_10);
                    } else {
                        magazine_motor.setVelocity(0.0f);
                        magazine_motor.setMaxVelocity(velocity_100);
                        robot_state = RobotState::BLIND_DRIVE;
                        state_timer.reset();
                        state_timer.start();
                    }
                    break;
                }
                case RobotState::BLIND_DRIVE: {
                // drive straight forward, ignore all sensors
                motor_left.setVelocity(drive_vel_rps);
                motor_right.setVelocity(drive_vel_rps);
                if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= blind_drive_ms) {
                    robot_state = RobotState::FIND_INTERSECTION;
                    printf("Searching for T-intersection...\r\n");
                    }
                    break;
                }
                case RobotState::FIND_INTERSECTION: {
                    // drive straight until outer sensors detect the horizontal line
                    motor_left.setVelocity(drive_vel_rps);
                    motor_right.setVelocity(drive_vel_rps);
                    float outer = lineFollower.getMeanFourAvgBitsOuter();
                    if (outer > outer_sensor_threshold)
                    {
                        robot_state = RobotState::ALIGN_PIVOT;
                        state_timer.reset();
                        printf("Intersection found (outer=%.2f), aligning...\r\n", outer);
                    }
                    break;
                }
                case RobotState::ALIGN_PIVOT: {
                    // drive forward a tiny bit more to center wheels on intersection
                    motor_left.setVelocity(drive_vel_rps);
                    motor_right.setVelocity(drive_vel_rps);
                    if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= align_pivot_ms)
                    {
                        robot_state = RobotState::TURN_LEFT_BLIND;
                        state_timer.reset();
                        printf("Pivoting left (blind phase)...\r\n");
                    }
                    break;
                }
                case RobotState::TURN_LEFT_BLIND: {
                    // point turn left: left backward, right forward
                    motor_left.setVelocity(-turn_vel_rps);
                    motor_right.setVelocity(turn_vel_rps);
                    if (duration_cast<milliseconds>(state_timer.elapsed_time()).count() >= turn_blind_ms)
                    {
                        robot_state = RobotState::TURN_LEFT_SEEK;
                        printf("Pivoting left (seeking line)...\r\n");
                    }
                    break;
                }
                case RobotState::TURN_LEFT_SEEK: {
                    // continue pivoting left slowly: left backward, right forward
                    motor_left.setVelocity(-turn_seek_vel_rps);
                    motor_right.setVelocity(turn_seek_vel_rps);
                    float center = lineFollower.getMeanFourAvgBitsCenter();
                    if (center > center_sensor_threshold)
                    {
                        robot_state = RobotState::LINE_FOLLOW;
                        printf("Line found (center=%.2f), switching to line follower\r\n", center);
                    }
                    break;
                }
                case RobotState::LINE_FOLLOW: {
                    printf("LINE_FOLLOW\n");
                    if (packages_placed==4){
                        robot_state=RobotState::FINISH;
                        break;
                    }
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
                            robot_state = RobotState::CROSS_LINE_STOP;
                            state_timer.reset();
                            printf("100mm line (outer=%.2f, center=%.2f), waiting %d ms\r\n",
                                outer_val, center_val, wait_duration_ms);
                        }
                        else if (center_val > cross_line_center_threshold && outer_val < cross_line_outer_threshold)
                        {
                            // 50mm line: center sensors strongly lit, outer NOT lit
                            placing = true;
                            wait_duration_ms = cross_line_wait_50mm_ms;
                            robot_state = RobotState::CROSS_LINE_STOP;
                            state_timer.reset();
                            printf("50mm line (outer=%.2f, center=%.2f), waiting %d ms\r\n",
                            outer_val, center_val, wait_duration_ms);
                        }
                    }
                    break;
                }   
                case RobotState::CROSS_LINE_STOP:{
                    printf("CROSS_LINE_STOP\n");
                    motor_left.setVelocity(0.0f);
                    motor_right.setVelocity(0.0f);
                    
                    robot_state = RobotState::CHECKING_COLOR;
                    break;
                }
                case RobotState::CHECKING_COLOR: {
                    printf("CHECKING_COLOR\n");
                    motor_left.setVelocity(0.0f);
                    motor_right.setVelocity(0.0f);

                    if (color_retry_counter > 0) {
                        color_retry_counter--;
                        break; // wait before retry
                    }

                    // read color
                    for (int j = 0; j < 4; j++) {
                        color_raw_Hz[j] = Color_Sensor.readRawColor()[j];
                    }

                    for (int j = 0; j < 4; j++) {
                        color_avg_Hz[j] = Color_Sensor.readColor()[j];
                    }

                    for (int j = 0; j < 4; j++) {
                        color_cal[j] = Color_Sensor.readColorCalib()[j];
                    }

                    color_num = Color_Sensor.getColor();
                    color_string = Color_Sensor.getColorString(color_num);

                    printf("Color Num: %d Color %s\n", color_num, color_string);

                    // color_num = Color_Sensor.getColor();
                    // color_string = Color_Sensor.getColorString(color_num);
                    // printf("Colour: %s\n", color_string);

                    // reset valid flag
                    color_valid = false;

                    switch (color_num) {
                        case 3: // RED
                            target_rotation = rotation_red;
                            color_valid = true;
                            break;
                        case 5: // GREEN
                            target_rotation = rotation_green;
                            color_valid = true;
                            break;
                        case 7: // BLUE
                            target_rotation = rotation_blue;
                            color_valid = true;
                            break;
                        case 4: // YELLOW
                            target_rotation = rotation_yellow;
                            color_valid = true;
                            break;

                        case 1: 
                        case 2: // BLACK / WHITE
                        default:
                            color_valid = false;
                            break;
                    }
                    if (color_valid) {
                        color_attempts = 0; // reset attempts
                        color_retry_counter = 0;

                        color_active = target_rotation;
                        magazine_motor.setMaxVelocity(velocity_100);
                        target_position_absolute = magazine_motor.getRotation() + target_rotation;
                        magazine_motor.setRotationRelative(target_rotation);

                        robot_state = RobotState::WAIT_FOR_MAGAZINE_INIT;
                    } 
                    else {
                        color_attempts++;

                        if (color_attempts >= max_color_retries) {
                            printf("No valid color after %d tries -> continue line follow\n", color_attempts);

                            color_attempts = 0;
                            color_retry_counter = 0;   // add this

                            cooldown_active = true;
                            cooldown_timer.reset();
                            cooldown_timer.start();

                            robot_state = RobotState::LINE_FOLLOW;
                        } else {
                            // retry after delay
                            color_retry_counter = color_retry_delay_cycles;
                        }
                    }
                    break;
                }
                case RobotState::WAIT_FOR_MAGAZINE_INIT: {
                    printf("%f ", magazine_motor.getRotation());                            // print the current position
                    printf("WAIT_FOR_MAGAZINE_INIT\n");                                     // print the state

                    // wait until target position is reached
                    if (fabs(magazine_motor.getRotation() - target_position_absolute) < positionTolerance) {
                        if (rePosNeeded) {
                            // Give repos command
                            robot_state = RobotState::REPOSITIONING;                        // next state
                        } else {
                            robot_state = RobotState::ARM_DOWN;                             // next state
                        }
                        
                    }
                    break;
                }
                case RobotState::REPOSITIONING: {

                    // wait for repositioning
                    robot_state = RobotState::ARM_DOWN;
                    
                    break;
                }
                case RobotState::ARM_DOWN:{
                    printf("ARM_DOWN\n");                                                   // print the state
                    magazine_motor.setMaxVelocity(velocity_20);     // set the velocity to 20%
                    target_rotation = -grip_offset;                                         // calculate the new target rotation
                    target_position_absolute = magazine_motor.getRotation() + target_rotation;  // calculate the absolut target position for later checks
                    magazine_motor.setRotationRelative(target_rotation);                    // command the motor
                    
                    robot_state = RobotState::WAIT_ARM_DOWN;

                    break;
                }
                case RobotState::WAIT_ARM_DOWN:{
                    printf("%f ", magazine_motor.getRotation());
                    printf("WAIT_ARM_DOWN\n");
                    if(fabs(magazine_motor.getRotation() - target_position_absolute) < positionTolerance){

                        
                        magazine_motor.setMaxVelocity(velocity_100);   // reset the velocity to 100%
                        // Delay, 25*20ms = 500ms
                        if (i<25){
                            i++;
                        }else{
                            i=0;
                            robot_state = RobotState::ARM_UP;
                        }
                        
                    }else{
                        i=0;
                    }

                    break;
                }
                case RobotState::ARM_UP:{
                    printf("ARM_UP\n");
                    magazine_motor.setMaxVelocity(velocity_20);
                    target_rotation = grip_offset;
                    target_position_absolute = magazine_motor.getRotation() + target_rotation;
                    magazine_motor.setRotationRelative(target_rotation);
                    
                    robot_state = RobotState::WAIT_ARM_UP;

                    break;
                }
                case RobotState::WAIT_ARM_UP:{
                    printf("%f ", magazine_motor.getRotation());
                    printf("WAIT_ARM_UP\n");
                    if(fabs(magazine_motor.getRotation() - target_position_absolute) < positionTolerance){

                        magazine_motor.setMaxVelocity(velocity_100);   // reset the velocity to 100%
                        // Delay, 25*20ms = 500ms
                        if (i<25){
                            i++;
                        }else{
                            i=0;
                            if (placing){
                                placing = false;
                                packages_placed ++;
                            } 
                            robot_state = RobotState::CHECK_PACKAGE;
                        }
                    }else{
                        i=0;
                    }

                    break;
                }
                case RobotState::CHECK_PACKAGE: {
                    printf("CHECK_PACKAGE\n");

                    /*
                     if(picking){
                        placing = false;
                        if(us_distance_cm > 0.1f && us_distance_cm < 2.0f){
                            packages_picked++;
                            success = true;
                        }else if(us_distance_cm < 0.0f){
                            success = false;
                            robot_state = RobotState::ARM_DOWN;
                        }
                    }
                    else if(placing){
                        picking = false;
                        if(us_distance_cm < 0.0f){
                            packages_placed++;
                            success = true;
                        }else if(us_distance_cm > 0.1f && us_distance_cm < 2.0f){
                            success = false;
                            robot_state = RobotState::ARM_DOWN;
                        }
                    }
                    */
                    // if pick place, package yes no
                    // increase the picked and place counters
                    // if all placed finish

                    // delete this after
                    success = true;
                    if(success){
                        success = false;
                        magazine_motor.setMaxVelocity(velocity_100);
                        target_rotation = (1.0-color_active);
                        target_position_absolute = magazine_motor.getRotation() + target_rotation;
                        magazine_motor.setRotationRelative(target_rotation);

                        robot_state = RobotState::WAIT_FOR_MAGAZINE;
                    }
                    break;
                }
                case RobotState::WAIT_FOR_MAGAZINE: {
                    printf("%f ", magazine_motor.getRotation());
                    printf("WAIT_FOR_MAGAZINE\n");
                    if(fabs(magazine_motor.getRotation() - target_position_absolute) < positionTolerance){

                        // Delay, 25*20ms = 500ms
                        if (i<25){
                            i++;
                        }else{
                            i=0;
                            robot_state = RobotState::LINE_FOLLOW;
                        }
                    }else{
                        i=0;
                    }

                    break;
                }
                case RobotState::FINISH: {
                    printf("FINISHED\n");
                    motor_left.setVelocity(0.0f);
                    motor_right.setVelocity(0.0f);
                    enable_motors = false;
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
            //led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                //led1 = 0;
                robot_state = RobotState::INITIAL;
                magazine_motor.setVelocity(0.0f);
                enable_motors = 0;
                color_valid = false;
                rePosNeeded = false;
                // stopDetected = 0;
                picking = false;
                placing = false;
                target_rotation = 0.0f;
                target_position_absolute = 0.0f;
                color_active    = 0.0f;
                i = 0;
                packages_picked = 0;
                packages_placed = 0;
                success = false;
                motor_left.setVelocity(0.0f);
                motor_right.setVelocity(0.0f);
                cooldown_active = false;
                wait_duration_ms = 0;
                state_timer.stop();
                state_timer.reset();
                cooldown_timer.stop();
                cooldown_timer.reset();
                color_retry_counter = 0;
                color_attempts = 0; 
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