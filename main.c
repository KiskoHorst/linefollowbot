#include <project.h>
#include <time.h>
#include <stdio.h>
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "I2C_made.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "IR.h"
#include "Ambient.h"
#include "Beep.h"

void mainloop();
void zumoloop();
void setState(int newState);
int min(int a, int b);
int max(int a, int b);
float minf(float a, float b);
void setSpeed(int left, int right);
int absolute(int input);

void reverse(float dir);
void enter();

/*
state 0: Line follower ready
state 1: Line follower positioning
state 2: Line follower waiting
state 3: Line follower driving

state 4: Zumo ready
state 5: Zumo positioning
state 6: Zumo waiting
state 7: Zumo entering arena
state 8: Zumo spin
state 9: Zumo charge
*/

int state = 0;
int lines = 0;
int targetlines;
int on_a_line = 0;
float speedMultiplier = 1.0;
struct sensors_ ref;

int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Ultra_Start();
    
    reflectance_start();
    IR_led_Write(1);
    ADC_Battery_Start();        
    int16 adcresult =0;
    float volts = 0.0;
    int battery = 1;
    
    
    time_t batterytimer = 0;
    int lastButton = 1;
    
    BatteryLed_Write(0);
    motor_start();
    motor_forward(0, 0);
    while (battery) //This loop keeps going as long as the batteries are not empty.
    {
        if (batterytimer > 8000)
        {
            batterytimer = 0;
            ADC_Battery_StartConvert();
            if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT))
            {
                //Measuring battery voltage based on ADC data.
                adcresult = ADC_Battery_GetResult16();
                volts = adcresult/546.0f;
            }
            
            if (volts < 4)
            {
                //Stop everything if battery voltage is below 4 volts
                battery = 0;
            }
        }
        batterytimer++;
        //printf("state:%d, line:%d", state, on_a_line);
        if (state == 1 || state == 3 || state == 5)
            mainloop();
        else if (state >= 7)
            zumoloop();
        else
        {
            int ir = IR_receiver_Read();
            if (ir != 1)
            {
                if (state == 2 || state == 6)
                {
                    setState(state+1);
                }
            }
            motor_forward(0,0);
        }
        if (!SW1_Read())
        {
            if (lastButton)
            {
                lastButton = 0;
                
                //The following code is run once every time the button is pressed
                
                reflectance_read(&ref);
    
                float ref_left = ref.l1/24000.0;
                float ref_right = ref.r1/24000.0;
                float ref_right2 = ref.r3/24000.0;
                float ref_left2 = minf(1.0, ref.l3/20690.0);
                
                if (ref_left > 0.9 && ref_right > 0.9 && ref_left2 > 0.9 && ref_right2 > 0.9 && (state == 0 || state == 4))
                {
                    /*
                    The button was pressed in wait mode while not on ground, switching modes
                    The battery light is used to signal the current mode
                    
                    Light on:  Zumo mode
                    Light off: Line follower mode
                    */
                    if (state == 0)
                    {
                        state = 4;
                        BatteryLed_Write(1);
                    }
                    else
                    {
                        state = 0;
                        BatteryLed_Write(0);
                    }
                }
                else
                {
                    if (state == 0 || state == 2 || state == 4 || state == 6)
                        setState(state+1);
                    else
                        if (state > 3)
                        {
                            setState(4);   
                        }
                        else
                        {
                            setState(0);   
                        }
                }
            }
        }
        else
        {
            lastButton = 1;   
        }
    }
    int led = 1;
    motor_stop();
    for (;;)
    {
        led = !led;
        BatteryLed_Write(led);
        CyDelay(200);
        //A flashing light means the batteries have run out.
    }
 }   

void setState(int newState) //Set state as well as other important variables
{
    state = newState;
    switch (state)
    {
        case 0:
            BatteryLed_Write(0);
            break;
        case 4:
            BatteryLed_Write(1);
            break;
        case 1:
            on_a_line = 0;
            speedMultiplier = 0.33;
            targetlines = 1;
            lines = 0;
            break;
        case 3:
            lines = 0;
            speedMultiplier = 1.0;
            targetlines = 2;
            break;
        case 5:
            on_a_line = 0;
            speedMultiplier = 0.33;
            targetlines = 1;
            lines = 0;
            break;
        case 7:
            speedMultiplier = 1.0;
            setSpeed(255,255);
            CyDelay(1000);
            break;
    }
    
    
}

void mainloop()
{
    reflectance_read(&ref);
    
    float ref_left = ref.l1/24000.0;
    float ref_right = ref.r1/24000.0;
    float ref_right2 = ref.r3/24000.0;
    float ref_left2 = minf(1.0, ref.l3/20690.0);
    
    //Determine the correct direction for following the line
    float dir = (1.7*(ref_left-ref_right)+ 2.2*(ref_left2-ref_right2))*255;
    
    if (ref_left > 0.8 && ref_right > 0.8 && ref_left2 > 0.8 && ref_right2 > 0.8)
    {
        //Detect a start/stop line, we are now on top of it
        on_a_line = 1;
        if(lines == targetlines -1 && state == 3)
        {
            setState(0); //Stop as soon as we touch the finish line in line follow mode
        }
    } else if (on_a_line && ref_left2 < 0.7 && ref_right2 < 0.7)
    {
        //We are no longer on top of a start/stop line
        lines += 1;
        if (lines == targetlines)
        {
            setState(state+1); //Advance the state as soon as we are past the required number of lines
        }
        on_a_line = 0;
    }
    setSpeed(256-dir, 255+dir);
}

void reverse(float dir) //Used by zumo mode to quickly get away from edges of the arena
{
    setSpeed(-255-dir, -255+dir);
    CyDelay(750);
    setSpeed(0,0);
    CyDelay(75); //Stop for a short time to avoid doing a wheelie
}
void zumoloop()
{
    reflectance_read(&ref);
    
    float ref_left = ref.l1/24000.0;
    float ref_right = ref.r1/24000.0;
    float ref_right2 = ref.r3/24000.0;
    float ref_left2 = minf(1.0, ref.l3/20690.0);
    
    //dir is used to dynamically adjust reversing direction away from edges
    float dir = (1.7*(ref_left-ref_right)+ 2.2*(ref_left2-ref_right2))*64;
    
    if (ref_left > 0.9 || ref_left2 > 0.9 || ref_right > 0.9 || ref_right2 > 0.9) {
        reverse(dir);   //Reverse quickly as soon as an edge is detected
    }
    
    if (state == 8) {
        setSpeed(-120, 120); //Spin in place to locate targets with ultrasound
    } else if (state == 9) {
        setSpeed(255,255);   //Target spotted, full speed ahead
    }
    float distance = Ultra_GetDistance();
    if (distance < 55) {
        state = 9;
    } else {
        state = 8;
    }
}

int min(int a, int b) //Returns the smaller integer
{
    if (a < b)
    return a;
    else
    return b;
}

float minf(float a, float b) //Returns the smaller float
{
    if (a < b)
    return a;
    else
    return b;
}

int max(int a, int b) //Returns the larger integer
{
    if (a > b)
    return a;
    else
    return b;
}

void setSpeed(int left, int right)
{
    /*
    Either left speed or right speed is always 255.
    This ensures we always move as fast as we can
    Unless speedMultiplier is less than 1. (used in positioning mode)
    */
    left = min(max(left, -255), 255)*speedMultiplier;
    right = min(max(right, -255), 255)*speedMultiplier;
    PWM_WriteCompare1(absolute(left));
    PWM_WriteCompare2(absolute(right));
    MotorDirLeft_Write(left < 0);
    MotorDirRight_Write(right < 0);
}

int absolute(int input) //Returns the absolute value of an integer
{
    return (input > 0)?input:-input;   
}