#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

void main() 
{
    int curTime =0;
    //open output log
    int file;
    FILE *fp;
    fp = fopen("MPL_data.txt","w");
	// Create I2C bus
	char *bus = "/dev/i2c-2";
	if((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, MPL115A2 I2C address is 0x60(96)
	ioctl(file, I2C_SLAVE, 0x60);

	// Reading Coefficents for compensation
	// Read 8 bytes of data from address(0x04)
	// A0 msb, A0 lsb, B1 msb, B1 lsb, B2 msb, B2 lsb, C12 msb, C12 lsb
    while(curTime < 5){
	    char reg[1] = {0x00};
	    write(file, reg, 1);
	    char data[8] = {0};
	    if(read(file, data, 8) != 8)
	    {
	    	printf("Error : Input/Output error \n");
	    	exit(1);
	    }

	    // Convert the data to floating points
	    float A0 = (data[0] * 256 + data[1]) / 8.0;
	    float B1 = (data[2] * 256 + data[3]) / 8192.0;
	    float B2 = (data[4] * 256 + data[5]) / 16384.0;
	    float C12 = ((data[6] * 256 + data[7]) / 4) / 4194304.0;
	
	    // Send Pressure measurement command(0x12)
	    // Start conversion(0x00)
	    char config[2] = {0};
	    config[0] = 0x12;
	    config[1] = 0x00;
	    write(file, config, 2);
	    sleep(1);
	
	    // Read 4 bytes of data from register(0x00)
	    // pres msb, pres lsb, temp msb, temp lsb
	    reg[0] = 0x00;
	    write(file, reg, 1);
	    if(read(file, data, 4) != 4)
	    {
		    printf("Error : Input/Output error \n");
		    exit(1);
	    }
	    else
	    {
	    	// Convert the data to 10-bits
	    	int pres = (data[0] * 256 + (data[1] & 0xC0)) / 64;
	    	int temp = (data[2] * 256 + (data[3] & 0xC0)) / 64;
		
		    // Calculate pressure compensation
		    float presComp = A0 + (B1 + C12 * temp) * pres - B2 * temp;

		    // Convert the data
		    float pressure = (65.0 / 1023.0) * presComp + 50.0;
		    float cTemp = (temp - 498) / (-5.35) + 25.0;
		    float fTemp = cTemp * 1.8 + 32.0;
    
	    	// Output data to screen
	    	printf("A0 is %.2f\n", A0);
	    	printf("B1 is %.2f\n", B1);
	    	printf("B2 is %.2f\n", B2);
	    	printf("C12 is %.4f\n", C12);
	    	printf("temp is %i\n", temp);
	    	printf("Pres is: %i\n", pres);
            printf("PresComp is: %.2f\n", presComp);
	    	printf("Pressure is : %.2f kPa \n", pressure);
	    	printf("Temperature in Celsius : %.2f C \n", cTemp);
	    	printf("Temperature in Fahrenheit : %.2f F \n", fTemp);

            //output to log
            fprintf(fp,"Time=%i seconds, Pressure=%.2f\n", curTime, pressure);
            curTime++;
	    }
    }
    fclose(fp);
}
