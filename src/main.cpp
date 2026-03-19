//colorsensor test

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "ColorSensor.h"
#include "Servo.h"

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

    // mechanical button
    DigitalIn mechanical_button(PC_5);  // create DigitalIn object to evaluate mechanical button, you
                                        // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);     // sets pullup between pin and 3.3 V, so that there
                                        // is a defined potential

    // color sensor
    float color_raw_Hz[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the measurement of the color sensor (in Hz)
    float color_avg_Hz[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the average measurement of the color sensor (in Hz)
    float color_cal[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the calibrated measurement of the color sensor
    
    int color_num = 0.0f; // define a variable to store the color number, e.g. 0 for red, 1 for green, 2 for blue, 3 for clear
    const char* color_string; // define a variable to store the color string, e.g. "red", "green", "blue", "clear"
    ColorSensor Color_Sensor(PB_3); // create ColorSensor object, connect the frequency output pin of the sensor to PB_3

    // servo
    float servo_input = 0.0f;
    float servo_D0_ang_min = 0.025f; // 0.5 ms
    float servo_D0_ang_max = 0.125f; // 2.5 ms
    Servo servo_D0(PB_D0, servo_D0_ang_min, servo_D0_ang_max);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // enable the servos
            if (!servo_D0.isEnabled())
                servo_D0.enable(servo_input);

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;

            if(mechanical_button.read()){
                // read the raw color measurement (in Hz) and store it in the defined variable
                for (int i = 0; i < 4; i++) {
                    color_raw_Hz[i] = Color_Sensor.readRawColor()[i]; // read the raw color measurement in Hz
                }
                
                // read the average color measurement (in Hz) and store it in the defined variable
                for (int i = 0; i < 4; i++) {
                    color_avg_Hz[i] = Color_Sensor.readColor()[i]; // read the average color measurement in Hz
                }

                // read the calibrated color measurement (unitless) and store it in the defined variable
                for (int i = 0; i < 4; i++) {
                    color_cal[i] = Color_Sensor.readColorCalib()[i];
                }

                // read the classified color number and store it in the defined variable
                color_num = Color_Sensor.getColor();

                // read the classified color string and store it in the defined variable
                color_string = Color_Sensor.getColorString(color_num);
            }


            // select servo target position
            switch (color_num) {
                case 3: // RED
                    servo_input = 0.25f;
                    break;
                case 5: // GREEN
                    servo_input = 0.5f;
                    break;
                case 7: // BLUE
                    servo_input = 0.75f;
                    break;
                case 4: // YELLOW
                    servo_input = 1.0f;
                    break;
                case 1: case 2: // BLACK, WHITE
                    servo_input = 0.0f;
                    break;
                default:
                    // servo_input = 0.0f;
                    break;
            }
            // command the servo
            servo_D0.setPulseWidth(servo_input);

            // printf("Color Raw Hz: %f %f %f %f\n", color_raw_Hz[0], color_raw_Hz[1], color_raw_Hz[2], color_raw_Hz[3]); // uncomment to print raw color measurement in Hz      
            // printf("Color Avg Hz: %f %f %f %f\n", color_avg_Hz[0], color_avg_Hz[1], color_avg_Hz[2], color_avg_Hz[3]); // uncomment to print average color measurement in Hz (used for calibration and color classification)
            // printf("Color Num: %d Color %s\n", color_num, color_string); // uncomment to print classified color number and string. careful: filters delay also delays the color classification,
                                                                         // so the first few readings after switching the color sensor might be wrong until the filters are settled           
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;

                for (int i = 0; i < 4; i++) {
                    color_raw_Hz[i] = 0.0f;
                    color_avg_Hz[i] = 0.0f;
                    color_cal[i] = 0.0f;
                }
                color_num = 0;
                color_string = nullptr;
                servo_D0.disable();
                // servo_input = 0.0f;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        // print Main Task state
                printf("MT: %d, ", do_execute_main_task);
            // print mechanical button state
                printf("btn: %d, ", mechanical_button.read());
            // print color
                printf("clr: %s, ", color_string);
            // print servo position
                printf("svo: %f\n", servo_input);
        

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