import time
import board
import busio
import adafruit_bmp3xx
import adafruit_adxl34x
import cv2 as cv
import pickle
import sys
import os
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
PATH = '/home/pi/PDF_launch/photos'

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
    for i in range(250):
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
        x = int(point[0]/400)+1
        y = int(point[1]/400)
        grid_space.append(chr(y+65)+str(x))
    top_placements = (Counter(grid_space).most_common(4))
    location_output = []
    for items in top_placements:
        location_output.append(items[0])
    return location_output

# BEN'S MATH STUFF (formerly tracking.h)

def getTheta(theta, WY, CY, WX, CX, R, time): # R is a constant, the radius of the rocket cross section
    sign = 1
    if (WY - CY) < 0:
        sign = -1

    return theta + (sign * sqrt((WX - CX) / R) * time) # Radians

def getTau(tau, CX, CY, theta, DtoCG, time):
    return tau + (0.5 * ((CX * cos(theta) - CY * sin(theta)) / DtoCG) * pow(time, 2)) # Might be better methods we'll see

def getPhi(phi, CX, CY, theta, DtoCG, time):
    return phi + (0.5 * ((CX * sin(theta) + CY * cos(theta)) / DtoCG) * pow(time, 2)) # Might be better methods we'll see

def recalibrateAcc(AX, AY, AZ, theta, phi): # Accounts for changing axes and converts g to ft/s^2
    GFTSS = 32.174
    AZ2 = (AZ - sin(phi)) * GFTSS
    AX2 = (AX - sin(theta) * cos(phi)) * GFTSS
    AY2 = (AY - cos(theta) * cos(phi)) * GFTSS

    return (AX2, AY2, AZ2)

def getVAndXY(V, X, Y, CZ, tau, phi, time):
    V2 = V + (CZ * time)
    X2 = X2 + (V * cos(phi) * cos(tau) * time)
    Y2 = Y2 + (V * cos(phi) * sin(tau) * time)

    return (V2, X2, Y2)

def getDeplV(VX, VY, V, phi, tau):
    VX2 = V * cos(phi) * cos(tau)
    VY2 = V * cos(phi) * sin(tau)

    return (VX2, VY2)

def predictGS(X, Y, VX, VY, weirdConst, WindSpeed, WindDir, time):
    # iterate time2 to account for time passed, pass same stuff for main parachute call
    VX2 = VX
    VY2 = VY
    X2 = X
    Y2 = Y
    for _ in range(26):
        VX2 -= weirdConst * time * pow(VX2 - WindSpeed * cos(WindDir), 2)
        VY2 -= weirdConst * time * pow(VY2 - WindSpeed * sin(WindDir), 2)
        X2 = VX2 * time
        Y2 = VY2 * time

    X2 += 2000
    Y2 += 2000

    gs = str(chr(int(X2 / 200) + 65)) + str(int(Y2 / 200))
    
    return (gs, X2, Y2, VX2, VY2)

def GetApproxV(VX, VY, weirdConst, WindSpeed, WindDir, time):
    VX2 = VX - weirdConst * time * pow(*VX - WindSpeed * cos(WindDir), 2)
    VY2 = VY - weirdConst * time * pow(*VY - WindSpeed * sin(WindDir), 2)

    return (VX2, VY2)

def GetXYafterAp(X, Y, VX, VY, time):
    return (VX * time, VY * time) # (X, Y) = return

def GetGridSquare(X, Y):
    X2 = X + 2000
    Y2 = Y + 2000

    return str(chr(int(X2 / 200) + 65)) + str(int(Y2 / 200))


# Set up MPL/BMP. Yes it's that fking easy
# Call mpl.pressure (not a function)

# i2c = busio.I2C(board.SCL, board.SDA)
# mpl = adafruit_mpl115a2.MPL115A2(i2c)
i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

# Set up stepper motor

motors = MotorKit(address=0x61)

# Set up ADXLs

adxl_main = adafruit_adxl34x.ADXL345(i2c, address=0x53)
adxl_alt = adafruit_adxl34x.ADXL345(i2c, address=0x1D)

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

# Ben's variables
theta = -3.14159 / 2.0
tau = -0.6981317 # user input
phi = 3.14159 / 2.0 # todo user input
radius = 0.25
DtoCG = 2.4375 # TODO: MAKE CORRECT
v = 0
X = 0
Y = 0
VX = 0
VY = 0

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

f = open('/home/pi/PDF_launch/' + str(int(time.time())) + '.txt', 'w')

f.write('Begin main()\n')

height_offset = 0.0
pressures = []      # list of size 10; add to back, pop front (older values towards beginning)

pressure = bmp.pressure / 10
for i in range(10 - 1): # list size of 2, fill all but one slot MAKE THIS 10 - 1 FOR ACTUAL MAIN.PY
    pressures.append(pressure) # ABSOLUTELY GET RID OF THE / 1000 FOR ACTUAL LAUNCH IF MPL READS KPA

f.write('Begin while()\n\n')

while True:
    if launched == 1 and (motor_p1 == 0 or motor_p2 == 1):
        motors.stepper2.onestep(style=stepper.DOUBLE, direction=stepper.BACKWARD) # janky ass method to keep motor retention

    pressure = bmp.pressure / 10 # SAME HERE AS ABOVE
    
    (x_main, y_main, z_main) = adxl_main.acceleration
    (x_alt, y_alt, z_alt) = adxl_alt.acceleration

    # THIS BLOCK DEFINITELY NEEDS TO CHANGE BASED ON THE POSITION OF THE SENSORS IN THE ROCKET
    # ALSO MAKE SURE YOU CHANGE THE DISTANCE TO THE CENTER OF GRAVITY
    x_main *= FT_PER_METER
    y_main *= FT_PER_METER
    z_main *= FT_PER_METER
    x_alt *= FT_PER_METER
    y_alt *= FT_PER_METER
    z_alt *= FT_PER_METER

    pressures.append(pressure)

    if not calibrated:
        height_offset = height_calibration(pressures[-1]) # ZEROED IN BASED ON THE FIRST PRESSURE VALUE READ IN SO YOU BETTER HOPE WE TURN THIS MF ON AT THE LAUNCH SITE'S ALTITUDE
        calibrated = 1

    height = actual_height_ft(pressure, height_offset) # HEIGHT ABOVE GROUND LEVEL
    previous_height = actual_height_ft(pressures[0], height_offset) # HEIGHT ABOVE GROUND LEVEL 1 SECOND AGO
    pressures.pop(0)

    # define ADXL vars here and read
    (x_main, y_main, z_main) = adxl_main.acceleration
    (x_alt, y_alt, z_alt) = adxl_alt.acceleration

    # ADJUST THIS BLOCK BASED ON WHAT BEN SAYS (BASED ON POSITION WITHIN THE ROCKET)
    x_main *= FT_PER_METER
    y_main *= FT_PER_METER
    z_main *= FT_PER_METER
    x_alt *= FT_PER_METER
    y_alt *= FT_PER_METER
    z_alt *= FT_PER_METER






    # ADJUST BASED ON ORIENTATION WITHIN ROCKET











    # 1. Launch
    #      1b. Based on sims 75 feet clears the resting pressure variance and is realistic to rocket flight
    if launched == 0 and (height - previous_height > 75):
        f.write('\n\nROCKET LAUNCHED\n\n')
        launched = 1 # launched

    # if launched, recalibrate accel for both ADXLs, get theta, tau, and phi, and get v and xy
    if launched == 1:
        (x_main, y_main, z_main, theta, phi) = recalibrateAcc(x_main, y_main, z_main, theta, phi)
        (x_alt, y_alt, z_alt, theta, phi) = recalibrateAcc(x_alt, y_alt, z_alt, theta, phi)
        theta = getTheta(theta, y_alt, y_main, x_alt, x_main, radius, TIME_INT)
        tau = getTau(tau, x_main, y_main, theta, DtoCG, TIME_INT)
        phi = getPhi(phi, x_main, y_main, theta, DtoCG, TIME_INT)
        (v, X, Y) = getVAndXY(v, X, Y, z_main, tau, phi, TIME_INT)

    # 2. Apogee
    if apogee == 0 and launched == 1 and height < previous_height:
        # get deplv
        f.write('\n\nAPOGEE\n\n')
        apogee = 1
        (VX, VY) = getDeplV(VX, VY, v, phi, tau)

    if apogee == 1 and motor_p1 == 0:
        (VX, VY) = GetApproxV(VX, VY, DROG_CHUTE_CONSTANT, WIND_SPEED, WIND_DIR, TIME_INT)
        (X, Y) = GetXYafterAp(X, Y, VX, VY, TIME_INT)

    if motor_p1 == 1 and height > 0:
        (VX, VY) = GetApproxV(VX, VY, MAIN_CHUTE_CONSTANT, WIND_SPEED, WIND_DIR, TIME_INT)
        (X, Y) = GetXYafterAp(X, Y, VX, VY, TIME_INT)

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
        gridSquare = predictGS(X, Y, VX, VY, MAIN_CHUTE_CONSTANT, WIND_SPEED, WIND_DIR, TIME_INT)
        f.write('Predicted Grid Square : ' + gridSquare + '\n\n')

    # 3.5. Taking a picture
    if motor_p1 == 1 and motor_p2 == 0 and filename < 10:
        if not timeSinceLaunch % 10: # every 5 seconds
            command = 'raspistill -o ' + PATH + '/' + str(filename) + '.jpg -t 1 --shutter 3000 -n' # save as %d.jpg
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
        f.write('\n\nLANDED\n\n')
        # getgridsquare
        AAAAAAA = GetGridSquare(X, Y)
        f.write('Actual Grid Square : ' + AAAAAAA + '\n\n')
        # write to file
        # send to arduino
        break

    curTime += 1
    if launched == 1:
        timeSinceLaunch += 1

    # Output data to screen
    f.write('\n')

    # output to log
    if not curTime % 5:
        f.write('Pressure: {} \tHeight: {}'.format(pressure, height))
        f.write('Main: ({}, {}, {}) \tAlt: ({}, {}, {})'.format(x_main, y_main, z_main, x_alt, y_alt, z_alt))

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

f.close()