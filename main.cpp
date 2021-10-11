#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include "altimeter.c"
#include "XY-Accel.c"


using namespace std;
 
#define I2C1_SCL "/sys/class/gpio/gpio19/direction";
#define I2C1_SDA "/sys/class/gpio/gpio20/direction";
#define DoorOpen "/sys/class/gpio/gpio66/direction";
#define DoorClose "/sys/class/gpio/gpio69/direction";

int main()
{
    int thresh = 1000000000;
    
    double height = -999;       
    
    double XAccel = -999;
    double YAccel = -999;
    
    double XCord = -1;
    double YCord = -1;
    double ZCord = -1;   ///May be useless
    
    
    DoorClose = 1;          //Door close stepper motor running. make sure no accidental opening
    DoorOpen = 0;           //Ensure that the motor isnt getting dueling signals

    
    while(1) //loop to fetch data
    {
        height = alt.getHeight();            //Altimeter STUFF

        XAccel = xyaccel.getXAccel();            //Accel X
        
        YAccel = xyaccel.getYAccel();            //Accel Y
        
        
        getTeleGPSCords();      //Telemetrum GPS
        
        //Lots of math
        
            
        if(height > thresh)
        {
            if(height < lastHeight)          //Maybe this depending on Tyler maths
            {
                //Trigger the camera door opening via GPIO pins
                DoorOpen = 1;           //Open the door via stepper motor
                DoorClose = 0;          //Ensure that the door is closed

            }
        }
        else //Close door
        {
            DoorClose = 1;          //Door close stepper motor running
            DoorOpen = 0;           //Make sure its not trying to force it open
        }
        
        
        //Tansfer data
        
        
        
    }
    
    
}