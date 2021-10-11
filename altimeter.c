#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h> 
#include <DEVICE LIBRARY.h>              //NEED THE CHIP CODE TO MAKE IT WORK
 
int altimeterAddr = 0x00;                //TODO fill in with correct
double lastHeight  = -9999999.99;        //To be compared with current height to detect if rocket is falling
int thresh = 100000000;                  //Value that the door should open


double getHeight()
{
    double height = -10.0;               //Init to a neg to ensure we catch a fatal reset fault
    
    //read in sensor to height
    //Maybe convert to human if needed
    
    return height;
}
