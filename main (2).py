import time
import board
import busio
import adafruit_bmp3xx
import cv2 as cv
import pickle
import sys
import os
import serial
from collections import Counter
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit

FT_PER_METER = 3.28084	# meters to feet
P0 = 101.325			# avg sea level pressure (KPa)
g = 9.80665			# g (m/s^2)
M = 0.02896968			# M = molar mass of dry air (kg/mol)
T0 = 288.16			# sea level std. temp (K)
R0 = 8.31432			# universal gas const (mol K)
L = -0.0065			# temperature lapse rate (K/m)

DISTANCE = 1.25           # distance belt needs to travel (inches)
NUM_ROTATIONS = (DISTANCE / (3.14159 * 0.46))    # number of rotations to move the motor out
STEPS_PER_ROTATION = 200

MAIN_CHUTE_CONSTANT = 0.15515088
DROG_CHUTE_CONSTANT = 0.01873845169
WIND_SPEED = 16.878 # in feet per second
WIND_DIR = 2.443460    # in radians (0 to 2Pi); E = 0; CCW; {Cos(t), Sin(t)}
TIME_INT = 0.1

DIRECTORY = 'photos'
PATH = '/home/ara/PDF_launch/photos'

# /* Calculates height(pressure) :
#  *		P = P0 * exp(-ghM/RT)
#  *	 :. h = ln(P_adj/P0) * (-RT/gM)
#  *		where P_adj is the adjusted pressure reading, using return val from pressure_calibration.
#  *		Subtract LAUNCH_ALT from h, return.
#  */

def height_calc(p):
    exp = -1 * (R0 * L) / (g * M)

    return (T0 / L) * (pow((p / P0), exp) - 1)


# /* Calibrates the pressure sensor :
#  *		Determines how inaccurate the pressure sensor is.
#  *		Calculates sea level pressure based on initial pressure & launch site height.
#  *		Compares result with universally-accepted sea level pressure.
#  *		Returns the difference (KPa).
#  */

def height_calibration(p):
    h0 = height_calc(p)

    return -1 * h0

def actual_height_ft(p, h_inv):
    return (height_calc(p) + h_inv) * FT_PER_METER

def rotate_motor(direction):
    # for i in range(NUM_ROTATIONS * STEPS_PER_ROTATION):
    for i in range(225):
        motors.stepper2.onestep(style=stepper.DOUBLE, direction=direction)

# Image recognition function

def findPossibleLocations(detection_image):
    img1 = cv.imread(detection_image,cv.IMREAD_GRAYSCALE)          # queryImage
    kp1, des1 = sift.detectAndCompute(img1,None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary
    flann = cv.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    correct = []
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.7*n.distance:
            correct.append(m)
    points = []
    for match in correct:
        p2 = kp2[match.trainIdx].pt
        points.append(p2)
    grid_space = []
    for point in points:
        x = str(int(point[0]/300)+1).zfill(2)
        y = str(int(point[1]/300)+65).zfill(2)
        grid_space.append(y+x)
    top_placements = (Counter(grid_space).most_common(1))
    location_output = []
    for items in top_placements:
        location_output.append(items[0])
    final_string = ""
    for items in location_output:
        final_string+=items
    return final_string

# Set up MPL. Yes it's that fucking easy
# Call mpl.pressure (not a function)

# i2c = busio.I2C(board.SCL, board.SDA)
# mpl = adafruit_mpl115a2.MPL115A2(i2c)
i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

# Set up stepper motor

motors = MotorKit(address=0x61)

# Set up ADXLs, maybe


# Initialize image recognition

index = pickle.loads(open("keypoints.txt", "rb").read())
des2 = pickle.loads(open("descriptors.txt", "rb").read())
img2 = cv.imread('grid.jpg',cv.IMREAD_GRAYSCALE)

kp2 = []

for point in index:
    temp = cv.KeyPoint(x=point[0][0],y=point[0][1],size=point[1], angle=point[2], response=point[3], octave=point[4], class_id=point[5])
    kp2.append(temp)
sift = cv.SIFT_create()

# main

calibrated = 0
curTime = 0
apogeeTime = 0
timeSinceLaunch = 0
launched = 0
landed = 0
apogee = 0
motor_p1 = 0
motor_p2 = 0
pressure = 0
filename = 0
outfile = 'LAUNCH_DAY_WOOOO'
    
f = open('/home/ara/PDF_launch/' + str(int(time.time())) + '.txt', 'a')

f.write('Begin main()\n')

height_offset = 0.0
pressures = []      # list of size 10; add to back, pop front (older values towards beginning)

pressure = bmp.pressure / 10
for i in range(10 - 1): # list size of 2, fill all but one slot MAKE THIS 10 - 1 FOR ACTUAL MAIN.PY
    pressures.append(pressure) # ABSOLUTELY GET RID OF THE / 1000 FOR ACTUAL LAUNCH IF MPL READS KPA

('Begin while()\n\n')

while True:
    pressure = bmp.pressure / 10 # SAME HERE AS ABOVE

    pressures.append(pressure)

    if not calibrated:
        height_offset = height_calibration(pressures[-1]) # ZEROED IN BASED ON THE FIRST PRESSURE VALUE READ IN SO YOU BETTER HOPE WE TURN THIS MF ON AT THE LAUNCH SITE'S ALTITUDE
        calibrated = 1
    
    height = actual_height_ft(pressure, height_offset) # HEIGHT ABOVE GROUND LEVEL
    previous_height = actual_height_ft(pressures[0], height_offset) # HEIGHT ABOVE GROUND LEVEL 1 SECOND AGO
    pressures.pop(0)

    # define ADXL vars here and read
    # if launched, recalibrate accel for both ADXLs, get theta, tau, and phi, and get v and xy

    # 1. Launch
    #      1b. Based on sims 75 feet clears the resting pressure variance and is realistic to rocket flight
    if launched == 0 and (height - previous_height > 75):
        f.write('\n\nROCKET LAUNCHED\n\n')
        launched = 1 # launched

    # 2. Apogee
    if apogee == 0 and launched == 1 and height < previous_height:
        # get deplv
        f.write('\n\nAPOGEE\n\n')
        apogee = 1

    # if (apogee == 1 && motor_p1 == 0) {
    #     GetApproxV(&VX, &VY, DROG_CHUTE_CONSTANT, WIND_SPEED, WIND_DIR, TIME_INT); // TODO replace constants
    #     GetXYafterAp(&X, &Y, VX, VY, TIME_INT);
    # }

    # if (motor_p1 == 1 && height > 0) {
    #     GetApproxV(&VX, &VY, MAIN_CHUTE_CONSTANT, WIND_SPEED, WIND_DIR, TIME_INT); // TODO replace constants
    #     GetXYafterAp(&X, &Y, VX, VY, TIME_INT);
    # }
    
    # 3. Main Parachute Deployment
    if motor_p1 == 0 and apogee == 1 and height <= 600: # needs to be <= because the chances of it being exactly 500 are almost guaranteed zero
        # // sticka da motor out
        # // motor object has to be created OUTSIDE of the while loop in main
        # // since the code will be running for a very long time we need to prevent the motor from overheating
        # // i don't think SLP is working as intended so might need to consider setting the enable bit high until launch condition (1) met
        # // also recall that the number of degrees needs to be doubled in order to get the desired number of rotations

        f.write('\n\nMAIN CHUTE\n\n')
        rotate_motor(stepper.FORWARD) # CHECK THIS, COULD BE stepper.BACKWARD TO BEGIN
        motor_p1 = 1
        # predictgs and write to file

    # 3.5. Taking a picture
    if motor_p1 == 1 and motor_p2 == 0 and filename < 10:
        if not timeSinceLaunch % 20: # every 2 seconds
            command = 'libcamera-still -o ' + PATH + '/' + str(filename) + '.jpg -t 1 --shutter 9000 -n --autofocus --width 2448 --height 3264' # save as %d.jpg
            os.system(command)
            filename += 1

    # 4. Landing
    if motor_p1 == 1 and motor_p2 == 0 and height <= 125:
        # bringa da motor in
        # use same degree parameter as main parachute deployment (3)
        f.write('\n\nMOTOR RETRACTION\n\n')

        rotate_motor(stepper.BACKWARD) # CHECK THIS, COULD BE stepper.FORWARD BUT IS THE REVERSE OF ABOVE
        motor_p2 = 1

    # 5. Actually Landed
    if abs(height - previous_height) < 30 and motor_p2 == 1:
        ('\n\nLANDED\n\n')
        # getgridsquare
        # write to file
        # send to arduino
        break

    curTime += 1
    if launched == 1:
        timeSinceLaunch += 1

    # Output data to screen
    f.write('\n')

    # output to log
    f.write('Pressure: {} \tHeight: {}'.format(pressure, height))
    
    time.sleep(0.1) # loop every decisecond


motors.stepper2.release() # at end of file

locations = []
for filename in os.listdir(PATH):
    f2 = os.path.join(PATH, filename)
    if os.path.isfile(f2):
        temp = findPossibleLocations(f2)
        f.write(str(temp))
        f.write('\n')
        locations.append(temp)

# send final location to arduino multiple times to ensure it transfers
# for _ in range(5):
#     for i in range(len(locations)):
#         # send final location to arduino
ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
ser.reset_input_buffer()
for dataLocations in locations:
    for sendingData in range(5):
        ser.write(bytes(dataLocations, 'UTF-8'))
        time.sleep(1)
    for sendingData in range(3):
        ser.write(bytes("1111", 'UTF-8'))
        time.sleep(1)

    

f.close()
