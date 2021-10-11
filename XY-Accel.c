#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h> 
#include <DEVICE LIBRARY.h>              //NEED THE CHIP CODE TO MAKE IT WORK

//MXC6255XU sensor is going to be X
 
int XAccelerometerAddr = 0x00;           //TODO fill in with correct


double getXAccel()
{
    double XAccel  = -9999999.99;         //Large neg value to throw exception if broken
    
    
    //Maybe convert to human if needed
    
    return XAccel;
}

double getYAccel()
{
    double YAccel = -3;
    return YAccel;
}
