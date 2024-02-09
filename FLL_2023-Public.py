# LEGO type:advanced slot:0 autostart
# from time import sleep
from spike import PrimeHub, ColorSensor, Motor, MotorPair
from spike.control import wait_for_seconds, Timer
# from hub import button
from util import time
from runtime.virtualmachine import VirtualMachine
import sys
import system
import hub
from hub import battery
from math import *
import math
import random
import math

phub = PrimeHub()
Robot = 2
# Ports
wheels = MotorPair('E', 'F')
colorA = ColorSensor('A') #left
colorB = ColorSensor('B') #right
motorC = Motor('C')
motorD = Motor('D')
# Variables
battery_percentage = 0
cancel = False
target_light = 58
KP = 0.25
adaptive = 1
LEFT = 1
RIGHT = 2
CENTER = 3
FORWARD = 1
BACKWARD = 2
DEFAULT_SPEED = 40
DEFAULT_TURN_SPEED = 20
CONFIG = 1
RUNNING = 0
motorC_Val = motorC.set_degrees_counted(0)
motorD_Val = motorD.set_degrees_counted(0)

#Robot info
circumference = 17.6  
sensordistance = 7 

#Preperation for parallel code execution
accelerate = True
run_generator = True
runSmall = True
lastAngle = 0
oldAngle = 0
gyroValue = 0

#PID value Definition
pRegler = 0.0
iRegler = 0.0
dRegler = 0.0
pReglerLight = 0.0
iReglerLight = 0.0
dReglerLight = 0.0



# All Routines

# Get correction value for line following based on color
def get_line_correction(reflected_light, sign):
    global integral, last_error
    error = reflected_light - target_light
    derivative = error - last_error
    integral = integral + error
    last_error = error
    if adaptive:
        correction = ((KP * error) + (derivative * 1) + (integral * 0.001)) * sign
    else:
        correction = KP * error * sign
    return correction

# Check if sensor detects black
def is_color_black(sensor):
    return sensor.get_reflected_light() < 25 and sensor.get_color() == 'black'

# Check if sensor detects white
def is_color_white(sensor):
    return sensor.get_reflected_light() > 95 and sensor.get_color() == 'white'

# Set sign, sensors for line following and color check
def get_line_follow_parameters(port, align, color='black'):
    if align == RIGHT:
        sign = -1
    else:
        sign = 1
    if port == 'A':
        line_sensor = colorA
        stop_sensor = colorB
    elif port == 'B':
        line_sensor = colorB
        stop_sensor = colorA
    if color == 'black':
        stop_func = is_color_black
    else:
        stop_func = is_color_white
    return sign, line_sensor, stop_sensor, stop_func

# Follow line using port, till other color sensor matches black or white
def line_follow_till_color(port, align, speed, color):
    global integral, last_error
    integral, last_error = 0, 0
    sign, line_sensor, stop_sensor, stop_func = get_line_follow_parameters(port, align, color)
    while True:
        if stop_func(stop_sensor):
            wheels.stop()
            break
        correction = get_line_correction(line_sensor.get_reflected_light(), sign)
        wheels.start_tank_at_power(speed + int(correction), speed - int(correction))

# Follow line using port, till other color sensor moves out of black or white
def line_follow_out_of_color(port, align, speed, color='black'):
    global integral, last_error
    integral, last_error = 0, 0
    sign, line_sensor, stop_sensor, stop_func = get_line_follow_parameters(port, align, color)
    while True:
        if not stop_func(stop_sensor):
            wheels.stop()
            break
        correction = get_line_correction(line_sensor.get_reflected_light(), sign)
        wheels.start_tank_at_power(speed + int(correction), speed - int(correction))

# Follow line using port, for duration (seconds)
def line_follow_timer(port, align, speed, duration):
    global integral, last_error
    integral, last_error = 0, 0
    sign, line_sensor, stop_sensor, stop_func = get_line_follow_parameters(port, align)
    msec = int(duration*1000)
    end_time = time.get_time() + msec
    while True:
        if time.get_time() >= end_time:
            wheels.stop()
            break
        correction = get_line_correction(line_sensor.get_reflected_light(), sign)
        wheels.start_tank_at_power(speed + int(correction), speed - int(correction))

# Drive for a distance in cm using move_tank
def drive_distance_cm(distance, direction, lspeed = DEFAULT_SPEED, rspeed = None):
    if rspeed == None:
        rspeed = lspeed
    if direction != FORWARD:
        lspeed *= -1
        rspeed *= -1
    wheels.move_tank(distance, 'cm', lspeed, rspeed)

# Drive forward till sensor detects black or white color
def drive_till_color(port, speed, color):
    if port == 'B':
        stop_sensor = colorB
    elif port == 'A':
        stop_sensor = colorA
    if color == 'black':
        stop_func = is_color_black
    else:
        stop_func = is_color_white
    wheels.start_tank(speed, speed)
    while not stop_func(stop_sensor):
        pass
    wheels.stop()
#Gets speed and pivot for turning

def get_motor_speeds_for_turn(pivot_point, side, speed):
    # Turn slower for CENTER pivot to reduce error
    if pivot_point == CENTER:
        speed = int(speed / 2)
    if pivot_point == LEFT:
        left_speed = 0
    else:
        left_speed = speed
    if pivot_point == RIGHT:
        right_speed = 0
    else:
        right_speed = -speed
    if side == LEFT:
        left_speed *= -1
        right_speed *= -1
    return left_speed, right_speed

# Turn using gyro to given angle
def gyro_turn(degrees, pivot_point, side, speed = DEFAULT_TURN_SPEED):
    left_speed, right_speed = get_motor_speeds_for_turn(pivot_point, side, speed)
    phub.motion_sensor.reset_yaw_angle()
    wait_for_seconds(0.5)
    wheels.start_tank(left_speed, right_speed)
    while True:
        if abs(phub.motion_sensor.get_yaw_angle()) >= degrees:
            wheels.stop()
            break
    phub.motion_sensor.reset_yaw_angle()

# Turn until sensor detects black or white color
def gyro_turn_till_color(pivot_point, side, port, color, speed = DEFAULT_TURN_SPEED):
    left_speed, right_speed = get_motor_speeds_for_turn(pivot_point, side, speed)
    phub.motion_sensor.reset_yaw_angle()
    if color == 'black':
        stop_func = is_color_black
    else:
        stop_func = is_color_white
    if port == 'A':
        stop_sensor = colorA
    elif port == 'B':
        stop_sensor = colorB
    wheels.start_tank(left_speed, right_speed)
    while True:
        if stop_func(stop_sensor):
            wheels.stop()
            break

#Moves straight using gyro for seconds
def gyro_move(duration, speed, direction):
    if direction == FORWARD:
        power = speed
        sign = 16
    elif direction == BACKWARD:
        power = -speed
        sign = -16
    phub.motion_sensor.reset_yaw_angle()
    msec = int(duration*1000)
    end_time = time.get_time() + msec
    while True:
        if time.get_time() >= end_time:
            wheels.stop()
            break
        correction = sign * (power * (phub.motion_sensor.get_yaw_angle() / 180))
        wheels.start_tank_at_power(power - int(correction), power + int(correction))
    phub.motion_sensor.reset_yaw_angle()

#Turning for coordinate System
def gyro_turn_for_coordinate(target_angle, pivot_point, side, speed = DEFAULT_TURN_SPEED):
    global current_orientation

    # Calculate the difference between the target and current angles
    diff_angle = (target_angle - current_orientation) % 360

    # Determine the direction of rotation
    if diff_angle > 180:
        diff_angle -= 360
    if diff_angle < 0:
        side = RIGHT
        diff_angle = -diff_angle
    else:
        side = LEFT

    left_speed, right_speed = get_motor_speeds_for_turn(pivot_point, side, speed)
    hub.motion_sensor.reset_yaw_angle()
    wheels.start_tank(left_speed, right_speed)

    while True:
        if abs(hub.motion_sensor.get_yaw_angle()) >= abs(diff_angle):
            wheels.stop()
            break

    current_orientation = target_angle

#Calculates angle for coordinate system
def calculate_angle(x1, y1, x2, y2):
    # Calculate the difference between the two points
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the angle in radians
    theta_radians = math.atan2(dy, dx)

    # Convert the angle to degrees
    theta_degrees = math.degrees(theta_radians)

    return theta_degrees

#Moves to the Coordinate Specified
def move_to_coordinate(X, Y):
    global positionx, positiony, angle
    angle = calculate_angle(positionx, positiony, X, Y)
    side1 = Y - positiony
    side2 = X - positionx
    distance = sqrt((side1 * side1) + (side2 * side2))

    if(angle - 0 < 0):
        gyro_turn_for_coordinate(angle, CENTER, LEFT)
    else:
        gyro_turn_for_coordinate(angle, CENTER, RIGHT)

    wheels.move_tank(distance, 'cm', left_speed=75, right_speed = 75)
    positionx = X
    positiony = Y
    current_orientation = angle

#Code from line 289 - 1125 excluding line 983 is from https://github.com/GO-Robot-FLL/Python-for-Spike-Prime
class DriveBase:

    def __init__(self, phub, leftMotor, rightMotor):
        self.phub = phub
        self.leftMotor = Motor(leftMotor)
        self.rightMotor = Motor(rightMotor)
        self.movement_motors = MotorPair(leftMotor, rightMotor) 

    def lineFollower(self, distance, startspeed, maxspeed, endspeed, sensorPort, side, addspeed = 0.2, brakeStart = 0.7 , stopMethod=None, generator = None, stop = True):
        """
            This is the function used to let the robot follow a line until either the entered distance has been achieved or the other sensor of the robot senses a line.
            Like all functions that drive the robot this function has linear acceleration and breaking. It also uses PID values that are automatically set depending on the
            current speed of the robot (See function PIDCalculationLight)
            Parameters
            -------------
            distance: The value tells the program the distance the robot has to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The percentage after which the robot reaches its maxspeed. Type: Float. Default: No default value
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: No default value
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """
        
        if cancel:
            return

        global run_generator, runSmall

        if generator == None:
            run_generator = False

        #set the speed the robot starts at
        speed = startspeed
        #reset PID values to eliminate bugs
        change = 0
        old_change = 0
        integral = 0
        #reset the driven distance of the robot to eliminate bugs

        #specifies the color sensor
        colorsensor = ColorSensor(sensorPort)
        #Get degrees of motors turned before robot has moved, allows for distance calculation without resetting motors
        loop = True
        #Going backwards is not supported on our robot due to the motors then being in front of the colour sensors and the program not working
        if distance < 0:
            print('ERR: distance < 0')
            distance = abs(distance)
        #Calculate target values for the motors to turn to
        finalDistance = (distance / 17.6) * 360
        #Calculate after what distance the robot has to reach max speed
        accelerateDistance = finalDistance * addspeed
        deccelerateDistance = finalDistance * (1 - brakeStart)

        invert = 1

        #Calculation of steering factor, depending on which side of the line we are on
        if side == "left":
            invert = 1
        elif side == "right":
            invert = -1
        
        #Calculation of the start of the robot slowing down
        
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        drivenDistance = getDrivenDistance(self)

        brakeStartValue = brakeStart * finalDistance
        while loop:
            if cancel:
                print("cancel")
                break
        
            if run_generator: #run parallel code execution
                next(generator)

            #Checks the driven distance as an average of both motors for increased accuracy
            oldDrivenDistance = drivenDistance
            drivenDistance = getDrivenDistance(self)
            #Calculates target value for Robot as the edge of black and white lines
            old_change = change

            change = colorsensor.get_reflected_light() - 50


            #Steering factor calculation using PID, sets new I value

        
            steering = (((change * pReglerLight) + (integral * iReglerLight) + (dReglerLight * (change - old_change)))) * invert
            integral = change + integral
            #Calculation of current speed for robot, used for acceleratiion, braking etc.
            speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)

            pidCalculationLight(speed)
            #PID value updates
            steering = max(-100, min(steering, 100))

            #Driving using speed values calculated with PID and acceleration for steering, use of distance check
            self.movement_motors.start_at_power(int(speed), int(steering))

            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
            else:   
                if finalDistance < drivenDistance:
                    break

        if stop:
            self.movement_motors.stop()
            
        run_generator = True
        runSmall = True
        generator = 0
        return

    def gyroRotation(self, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, rotate_mode = 0, stopMethod = None, generator = None, stop = True):
        """
            This is the function that we use to make the robot turn the length of a specific angle or for the robot to turn until it senses a line. Even in this function the robot
            can accelerate and slow down. It also has Gyrosensor calibrations based on our experimental experience.
            Parameters
            -------------
            angle: The angle which the robot is supposed to turn. Use negative numbers to turn counterclockwise. Type: Integer. Default value: No default value
            startspeed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The percentage after which the robot reaches the maxspeed. Type: Float. Default: No default value
            brakeStart: The percentage after which the robot starts slowing down until it reaches endspeed. Type: Float. Default: No default value
            rotate_mode: Different turning types. 0: Both motors turn, robot turns on the spot. 1: Only the outer motor turns, resulting in a corner. Type: Integer. Default: 0
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """
        #phub.motion_sensor.reset_yaw_angle()
        if cancel:
            return

        global run_generator, runSmall

        if generator == None:
            run_generator = False

        if rotate_mode == 0:
            startspeed = abs(startspeed)
            maxspeed = abs(maxspeed)
            endspeed = abs(endspeed)

        speed = startspeed

        #set standard variables
        rotatedDistance = 0
        steering = 1

        accelerateDistance = abs(angle * addspeed) 
        deccelerateDistance = abs(angle * (1 - brakeStart))

        #gyro sensor calibration
        angle = angle * (2400/2443) #experimental value based on 20 rotations of the robot

        #Setting variables based on inputs
        loop = True
        gyroStartValue = getGyroValue() #Yaw angle used due to orientation of the self.phub. This might need to be changed
        brakeStartValue = (angle + gyroStartValue) * brakeStart

        #Inversion of steering value for turning counter clockwise
        if angle < 0:
            steering = -1

        #Testing to see if turining is necessary, turns until loop = False
        while loop:
            if cancel:
                break

            if run_generator: #run parallel code execution
                next(generator)

            oldRotatedDistance = rotatedDistance
            rotatedDistance = getGyroValue() #Yaw angle used due to orientation of the self.phub. This might need to be changed
            speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, abs(1), abs(0))
            
            
            #Checking for variants
            #Both Motors turn, robot moves on the spot
            if rotate_mode == 0:
                self.movement_motors.start_tank_at_power(int(speed) * steering, -int(speed) * steering)
            #Only outer motor turns, robot has a wide turning radius
            
            elif rotate_mode == 1:

                if angle * speed > 0:
                    self.leftMotor.start_at_power(- int(speed))
                else:
                    self.rightMotor.start_at_power(+ int(speed))

            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
                    break
            elif abs(angle) <= abs(rotatedDistance - gyroStartValue):                   
                    loop = False
                    break



        #Stops movement motors for increased accuracy while stopping
        if stop:
            self.movement_motors.stop()
        run_generator = True
        runSmall = True

        return # End of gyroStraightDrive

    def gyroStraightDrive(self, distance, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, stopMethod=None, offset = 0, generator = None, stop = True):
        """
            This is the function that we use to make the robot go forwards or backwards without drifting. It can accelerate, it can slow down and there's also PID. You can set the values
            in a way where you can either drive until the entered distance has been achieved or until the robot senses a line.
            Parameters
            -------------
            distance: the distance that the robot is supposed to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
            port: This value tells the program whether the robot is supposed to check for a black line with the specified light snsor. Type: String. Default: 0
            lightValue: This value tells the program the value the robot should stop at if port sees it. Type: Integer. Default: 0
            align_variant: Tells the robot to align itself to a line if it sees one. 0: No alignment. 1: standard alignment. 2: tangent based alignment Type: Integer. Default: 0
            detectLineStart: The value which we use to tell the robot after what percentage of the distance we need to look for the line to drive to. Type: Float. Default: 0
            offset: The value sends the robot in a direction which is indicated by the value entered. Type: Integer. Default: 0
            generator: Function executed while robot is executing gyroStraightDrive. Write the wanted function and its parameters here. Type: . Default: 0
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """

        if cancel:
            return

        global run_generator, runSmall
        global pRegler, iRegler, dRegler
        
        if generator == None:
            run_generator = False

        #Set starting speed of robot
        speed = startspeed
        #Sets PID values

        change = 0
        old_change = 0
        integral = 0
        steeringSum = 0

        invert = -1

        #Sets values based on user inputs
        loop = True


        gyroStartValue = getGyroValue()

        #Error check for distance
        if distance < 0:
            print('ERR: distance < 0')
            distance = abs(distance)

        #Calulation of degrees the motors should turn to
        #17.6 is wheel circumference in cm. You might need to adapt it
        rotateDistance = (distance / 17.6) * 360
        accelerateDistance = rotateDistance * addspeed
        deccelerateDistance = rotateDistance * (1 - brakeStart)

        #Inversion of target rotation value for negative values
        if speed < 0:
            invert = 1

        #Calculation of braking point
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        brakeStartValue = brakeStart * rotateDistance
        drivenDistance = getDrivenDistance(self)

        while loop:
            if cancel:
                break
            if run_generator: #run parallel code execution
                next(generator)

            #Calculation of driven distance and PID values
            oldDrivenDistance = drivenDistance
            drivenDistance = getDrivenDistance(self)

            pidCalculation(speed)
            change = getGyroValue() - gyroStartValue #yaw angle used due to orientation of the self.phub


            currenSteering = (change * pRegler + integral * iRegler + dRegler * (change - old_change)) + offset + steeringSum*0.02

            currenSteering = max(-100, min(currenSteering, 100))
            #print("steering: " + str(currenSteering) + " gyro: " + str(change) + " integral: " + str(integral))

            steeringSum += change
            integral += change - old_change
            old_change = change

            #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight
            speed = speedCalculation(speed, startspeed,maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)
            self.movement_motors.start_at_power(int(speed), invert * int(currenSteering))


            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
            elif rotateDistance < drivenDistance:                   
                    loop = False


        if stop:
            self.movement_motors.stop()

        run_generator = True
        runSmall = True
        
        return #End of gyroStraightDrive

    def arcRotation(self, radius, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, stopMethod=None, generator = None, stop = True):  
        """
            This is the function that we use to make the robot drive a curve with a specified radius and to a given angle
            Parameters
            -------------
            radius: the radius of the curve the robot is supposed to drive; measured from the outside edge of the casing. Type: Integer. Default: 0
            angle: the angle that the robot is supposed to rotate on the curve. Type: Integer. Default: 0
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """

        if cancel:
            print("cancel")
            return

        global run_generator, runSmall

        if generator == None:
            run_generator = False

        angle = angle * (2400/2443) #gyro calibration
        
        gyroStartValue = getGyroValue()
        finalGyroValue = gyroStartValue + angle
        currentAngle = gyroStartValue

        accelerateDistance = abs(angle * addspeed)
        deccelerateDistance = abs(angle * (1 - brakeStart))
        brakeStartValue = abs(angle * brakeStart)

        loop = True

        #Calculating the speed ratios based on the given radius
        if angle * startspeed > 0:
            speed_ratio_left = (radius+14) / (radius+2) #calculate speed ratios for motors. These will need to be adapted based on your robot design
            speed_ratio_right = 1
        else:
            speed_ratio_left = 1
            speed_ratio_right = (radius+14) / (radius+2)
        
        #Calculating the first speed to drive with
        left_speed = speedCalculation(startspeed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)
        right_speed = speedCalculation(startspeed, startspeed , maxspeed , endspeed , accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)
        while loop:
            #when the cancel button is pressed stop the gyrostraight drive directly
            if cancel:
                break

            if run_generator: #run parallel code execution
                next(generator)

            currentAngle = getGyroValue() #Yaw angle used due to orientation of the self.phub. This might need to be changed
        
            #Calculating the current speed the robot should drive
            left_speed = speedCalculation(left_speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)
            right_speed = speedCalculation(right_speed, startspeed , maxspeed , endspeed , accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)


            self.movement_motors.start_tank_at_power(int(left_speed* speed_ratio_left), int(right_speed* speed_ratio_right))
        
            #if there is a stopMethod passed use it and stop the loop if it returns true otherwise check if the robot has rotated to the given angle
            if stopMethod != None:
                #print("stoMeth")
                if stopMethod.loop():
                    loop = False
                    break

            (angle / abs(angle))
            if finalGyroValue * (angle / abs(angle)) < currentAngle * (angle / abs(angle)):
                #print("finalGyroValue: " + str(finalGyroValue) + " rotatedDistance: " + str(currentAngle))                  
                loop = False
                break


            

        #if stop is true then stop the motors otherwise don't stop the motor
        if stop:
            self.movement_motors.stop()

        run_generator = True
        runSmall = True
        return #End of arcRotation

def resetGyroValue():
    global gyroValue
    hub.motion.yaw_pitch_roll(0)

    gyroValue = 0

def getGyroValue():

    #this method is used to return the absolute gyro Angle and the angle returned by this method doesn't reset at 180 degree
    global lastAngle
    global oldAngle
    global gyroValue

    #gets the angle returned by the spike prime program. The problem is the default get_yaw_angle resets at 180 and -179 back to 0
    angle = phub.motion_sensor.get_yaw_angle()

    if angle != lastAngle:
        oldAngle = lastAngle
        
    lastAngle = angle

    if angle == 179 and oldAngle == 178:
        hub.motion.yaw_pitch_roll(0)#reset
        gyroValue += 179
        angle = 0
    
    if angle == -180 and oldAngle == -179:
        hub.motion.yaw_pitch_roll(0) #reset
        gyroValue -= 180   
        angle = 0

    return gyroValue + angle

def getDrivenDistance(data):


    #print(str(abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue)) + " .:. " + str(abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)))

    drivenDistance = (
                    abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue) + 
                    abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)) / 2

    return drivenDistance

def defaultClass(object, db):
    object.db = db
    object.leftMotor = db.leftMotor
    object.rightMotor = db.rightMotor

    object.left_Startvalue = abs(db.leftMotor.get_degrees_counted())
    object.right_Startvalue = abs(db.rightMotor.get_degrees_counted())
    return object

class stopMethods(): #This class has all our stopmethods for easier coding and less redundancy
    
    class stopLine():
        """
            Drive until a Line is detected
            Parameters
            -------------
            db: the drivebase of the robot
            port: Port to detect line on
            lightvalue: Value of the light to detect
            detectLineDistance: Distance until start detecting a line
            """
        def __init__(self, db, port, lightvalue, detectLineDistance):
            self = defaultClass(self, db)            

            self.port = port
            self.detectLineDistance = (detectLineDistance / 17.6) * 360

            #if lightvalue bigger 50 stop when lightvalue is higher
            self.lightvalue = lightvalue


        def loop(self):


            drivenDistance = getDrivenDistance(self)

            if abs(self.detectLineDistance) < abs(drivenDistance):
                if self.lightvalue > 50:
                    if ColorSensor(self.port).get_reflected_light() > self.lightvalue:
                        return True
                else:
                    if ColorSensor(self.port).get_reflected_light() < self.lightvalue:
                        return True

            return False
    
    class stopAlign():
        """
            Drive until a Line is detected
            Parameters
            -------------
            db: the drivebase of the robot
            port: Port to detect line on
            lightvalue: Value of the light to detect
            speed: speed at which the robot searches for other line
            """
        def __init__(self, db, lightvalue, speed):
            self = defaultClass(self, db)    
            self.speed = speed


            #if lightvalue bigger 50 stop when lightvalue is higher
            self.lightValue = lightvalue


        def loop(self):

            if colorA.get_reflected_light() < self.lightValue:
                self.rightMotor.stop()
                #Turning robot so that other colour sensor is over line
                while True:

                    self.leftMotor.start_at_power(-int(self.speed))

                    #Line detection and stopping
                    if colorB.get_reflected_light() < self.lightValue or cancel:
                        self.leftMotor.stop()
                        return True
                

            #Colour sensor F sees line first
            elif colorB.get_reflected_light() < self.lightValue:

                self.leftMotor.stop()

                #Turning robot so that other colour sensor is over line
                while True:
                    self.rightMotor.start_at_power(int(self.speed))

                    #Line detection and stopping
                    if colorA.get_reflected_light() < self.lightValue or cancel:
                        self.rightMotor.stop()
                        return True
            

            return False

    class stopTangens():
        """
            Drive until a Line is detected
            Parameters
            -------------
            db: the drivebase of the robot
            port: Port to detect line on
            lightvalue: Value of the light to detect
            speed: Distance until start detecting a line
            """
        def __init__(self, db, lightvalue, speed):
            self.count = 0
            self = defaultClass(self, db)    
            self.speed = speed
            #if lightvalue bigger 50 stop when lightvalue is higher
            self.lightValue = lightvalue
            self.detectedLineDistance = 0

            self.invert = 1
            if speed < 0:
                self.invert = -1
            
        def loop(self):
            drivenDistance = getDrivenDistance(self)
            if colorA.get_reflected_light() < self.lightValue:
                    #measuring the distance the robot has driven since it has seen the line
                    if(self.detectedLineDistance == 0):
                        self.detectedLineDistance = getDrivenDistance(self)
                        self.detectedPort = 'E'

                    elif self.detectedPort == 'F':
                        db.movement_motors.stop() #Stops robot with sensor F on the line
                        angle = math.degrees(math.atan(((drivenDistance - self.detectedLineDistance) / 360 * circumference) / sensordistance)) #Calculating angle that needs to be turned using tangent
                        #print("angle: " + str(angle))
                        db.gyroRotation(-angle, self.invert * self.speed, self.invert * self.speed, self.invert * self.speed, rotate_mode=1) #Standard gyrorotation for alignment, but inverting speed values if necessary

                        db.movement_motors.stop() #Stopping robot for increased reliability
                        return True

                #Colour sensor F sees line first
            elif colorB.get_reflected_light() < self.lightValue:
                #measuring the distnace the robot has driven since it has seen the line
                if(self.detectedLineDistance == 0):
                    self.detectedLineDistance = drivenDistance
                    self.detectedPort = 'F'

                elif self.detectedPort == 'E':
                    db.movement_motors.stop() #Stops robot with sensor E on the line
                    angle = math.degrees(math.atan(((drivenDistance - self.detectedLineDistance) / 360 * circumference) / sensordistance)) #Calculation angle that needs to be turned using tangent
                    db.gyroRotation(angle, self.invert * self.speed, self.invert * self.speed, self.invert * self.speed, rotate_mode=1) #Standard gyrorotation for alignment, but inverting speed values if necessary
                    db.movement_motors.stop() #Stopping robot for increased reliablity
                    return True

            return False
    class stopDegree():
        """
            Roates until a certain degree is reached
            Parameters            
            -------------
            db: the drivebase of the robot
            angle: the angle to rotate
        """
        def __init__(self, db, angle):
            self.angle = angle * (336/360)
            
            self.gyroStartValue = getGyroValue() #Yaw angle used due to orientation of the self.phub.
            

        def loop(self):
            rotatedDistance = getGyroValue() #Yaw angle used due to orientation of the self.phub. 

            if abs(self.angle) <= abs(rotatedDistance - self.gyroStartValue):
                return True
            else:
                return False

    class stopTime():

        """
            Drive until a certain time is reached
            Parameters
            -------------
            db: the drivebase of the robot
            time: the time to drive
        """

        def __init__(self, db, time) -> None:
            self = defaultClass(self, db)
            self.time = time
            self.timer = Timer()
            self.startTime = self.timer.now()

        def loop(self):
            if self.timer.now() > self.startTime + self.time:
                return True
            else:
                return False
       
    class stopResistance():

        """
            Drive until the Robot doesn't move anymore
            Parameters
            -------------
            db: the drivebase of the robot
            restistance: the value the resistance has to be below to stop              
        """
        def __init__(self, db, resistance):
            self = defaultClass(self, db)
            self.resistance = resistance
            self.timer = Timer()
            
            self.startTime = 0
            self.lower = False
            self.runs = 0

        def loop(self):

            self.runs += 1
            motion = abs(hub.motion.accelerometer(True)[2])
            if motion < self.resistance:
                self.lower = True

            if self.runs > 15:
                if self.lower:
                    if self.startTime == 0:
                        self.startTime = self.timer.now()

                    if self.timer.now() > self.startTime:
                        return True

                else:
                    self.lower = False
                    return False
                
SM = stopMethods()

def motorResistance(speed, port, resistancevalue):
    """
    lets the motor stop when it hits an obstacle
    """
    if abs(resistancevalue) > abs(speed):
        return

        
    if cancel:
        return

    if port == "A":
        motorC.start_at_power(speed)
        while True:
            old_position = motorC.get_position()
            wait_for_seconds(0.4)
            if abs(old_position - motorC.get_position())<resistancevalue or cancel:
                motorC.stop()
                print("detected stalling")
                return

    elif port == "D":
        motorD.start_at_power(speed)
        while True:
            old_position = motorD.get_position()
            wait_for_seconds(0.4)
            if abs(old_position - motorD.get_position())<resistancevalue or cancel:
                motorD.stop()
                print("detected stalling")
                return
    else:
        print("wrong port selected. Select A or D")
        return

def speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance):
    """
        Used to calculate all the speeds in out programs. Done seperatly to reduce redundancy. Brakes and accelerates
        Parameters
        -------------
        speed: The current speed the robot has
        startspeed: Speed the robot starts at. Type: Integer. Default: No default value.
        maxspeed: The maximum speed the robot reaches. Type: Integer. Default: No default value.
        endspeed: Speed the robot aims for while braking, minimum speed at the end of the program. Type: Integer. Default: No default value.
        addspeed: Percentage of the distance after which the robot reaches the maximum speed. Type: Integer. Default: No default value.
        brakeStartValue: Percentage of the driven distance after which the robot starts braking. Type: Integer. Default: No default value.
        drivenDistance: Calculation of the driven distance in degrees. Type: Integer. Default: No default value.
    """    

    addSpeedPerDegree = (maxspeed - startspeed) / accelerateDistance 
    subSpeedPerDegree = (maxspeed - endspeed) / deccelerateDistance
    

    subtraction = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * subSpeedPerDegree
    addition = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * addSpeedPerDegree

    if abs(drivenDistance) > abs(brakeStartValue):

        if abs(speed) > abs(endspeed):
            speed = speed - subtraction
            
    elif abs(speed) < abs(maxspeed):

        speed = speed + addition

    return speed

def breakFunction(args):
    """
    Allows you to manually stop currently executing round but still stays in main. 
    This is much quicker and more reliable than pressing the center button.
    """
    global cancel, inMain
    if not inMain:
        cancel = True

def pidCalculation(speed):
    #golbally sets PID values based on current speed of the robot, allows for fast and accurate driving
    global pRegler
    global iRegler
    global dRegler
    #Important note: These PID values are experimental and based on our design for the robot. You will need to adjust them manually. You can also set them statically as you can see below
    if speed > 0:
        pRegler = -0.17 * speed + 12.83
        iRegler = 12
        dRegler = 1.94 * speed - 51.9
        if pRegler < 3.2:
            pRegler = 3.2
    else:
        pRegler = (11.1 * abs(speed))/(0.5 * abs(speed) -7) - 20
        iRegler = 10
        #iRegler = 0.02
        dRegler = 1.15**(- abs(speed)+49) + 88
    
def pidCalculationLight(speed):
    #Sets the PID values for the lineFollower based on current speed. Allows for accurate and fast driving
    #Important note: these PID values are experimental and based on our design for the robot. You will need to adjust them. See above on how to do so
    global pReglerLight
    global dReglerLight

    pReglerLight = -0.04 * speed + 4.11
    dReglerLight = 0.98 * speed - 34.2
    #set hard bottom for d value, as otherwise the values don't work
    if dReglerLight < 5:
        dReglerLight = 5

def driveMotor(rotations, speed, port):
    """
    Allows you to drive a small motor in parallel to driving with gyroStraightDrive
    Parameters
    -------------
    rotations: the rotations the motor turns
    speed: the speed at which the motor turns
    port: the motor used. Note: this cannot be the same motors as configured in the motor Drivebase
    """
           
    global runSmall
    global run_generator

    if cancel:
        runSmall = False
        run_generator = False

    while runSmall:
        smallMotor = Motor(port)
        smallMotor.set_degrees_counted(0)

        loop_small = True
        while loop_small:
            drivenDistance = smallMotor.get_degrees_counted()
            smallMotor.start_at_power(speed)
            if (abs(drivenDistance) > abs(rotations) * 360):
                loop_small = False
            if cancel:
                loop_small = False
            yield

        smallMotor.stop()
        runSmall = False
        run_generator = False
    yield

hub.motion.yaw_pitch_roll(0)

db = DriveBase(phub, 'E', 'F')

#Moves arm up
def arm_up(degrees=None,speed=DEFAULT_SPEED):
    if not degrees:
        degrees = motorD.get_degrees_counted()
    motorD.run_for_degrees(-degrees, speed)

#Moves arm down
def arm_down(degrees=None,speed=DEFAULT_SPEED):
    if not degrees:
        degrees = motorD.get_degrees_counted()
    motorD.run_for_degrees(degrees, speed)

#Turns MotorC for degrees
def MotorC_Run(degrees, speed=100):
    motorC.run_for_degrees(-degrees, speed)

#Missions
def Pick_Up_Skateboard_Expert_New():
    db.gyroRotation(44, 70, 70, 70, rotate_mode=1)
    db.gyroStraightDrive(30, 40, 70, 40)
    arm_up(200, 20)

def Chicken_Printer_New():
    drive_distance_cm(15, FORWARD, 40, 40)
    #wait_for_seconds(0.5)
    MotorC_Run(-1600, 100)
    db.gyroStraightDrive(29, -40, -70, -40)

def Collet_VE_Director_New():
    db.gyroRotation(-45, 60, 60, 60)
    #Adjust as needed
    db.arcRotation(15, 25, 50, 50, 50)
    db.gyroStraightDrive(13, 60, 60, 60)

def Tower_New():
    db.gyroRotation(-23, 60, 60, 60)
    db.gyroStraightDrive(65, 30, 100, 30)
    db.gyroRotation(-35, 60, 60, 60)
    db.gyroStraightDrive(20, -30, -100, -30)
    db.gyroStraightDrive(18, -30, -60, -30, stopMethod=SM.stopResistance(db, 1500))
    MotorC_Run(2050, 100)
    drive_distance_cm(90, FORWARD, 100, 100)

def Art_Piece_Delivery_New():
    drive_distance_cm(2, BACKWARD, 40, 40)
    db.gyroStraightDrive(45, 30, 90, 30)
    db.arcRotation(20, -22, 40, 60, 40)
    #Adjust as needed
    db.gyroStraightDrive(33, 40, 90, 40)
    
def Immersive_Experience_New():
    #Adjust as needed
    db.gyroStraightDrive(25, -40, -90, -40)
    db.gyroRotation(-27, 60, 60, 60)
    drive_distance_cm(28, FORWARD, 60, 60)
    drive_distance_cm(10, BACKWARD, 60, 60)
    drive_distance_cm(12, FORWARD, 60, 63)
    #Adjust as needed
    drive_distance_cm(14, BACKWARD, 40, 40)

def Stage_Push_Lights_New():
    db.gyroRotation(79, 80, 80, 80)
    arm_down(160, 30)
    db.gyroStraightDrive(53, 30, 90, 30, 0.1, 0.9)
    db.gyroRotation(-12, 80, 80, 80)
    drive_distance_cm(18, FORWARD, 60, 60)
    db.gyroStraightDrive(11, -50, -50, -50)
    db.gyroRotation(70, 80, 80, 80)
    drive_distance_cm(60, FORWARD, 100, 100)

def Spinning_Show__Pick_Up_Expert_New():
    arm_down(200)
    drive_distance_cm(2, BACKWARD, 40, 40)
    db.gyroStraightDrive(57, 30, 100, 30)
    arm_up(200, 15)
    db.gyroRotation(15, 60, 60, 60)
    db.gyroStraightDrive(2.5, 40, 40, 40)
    db.gyroRotation(-15, 40, 40, 40)
    drive_distance_cm(3, FORWARD, 0, 20)

def Deliver_Skateboard_Expert_New():
    #Adjust This Next Line
    db.gyroRotation(52, 60, 60, 60)
    db.gyroStraightDrive(19, 50, 70, 50)
    db.arcRotation(35, -25, -50, -50, -50)
    drive_distance_cm(90, BACKWARD, 95, 100)

def Sound_Mixer_New():
    drive_distance_cm(2, BACKWARD, 30, 30)
    db.gyroStraightDrive(48, 30, 100, 60, brakeStart=0.6)
    wait_for_seconds(0.5)
    drive_distance_cm(53, BACKWARD, 60, 65)
    
def Rollling_Cam():
    #db.gyroStraightDrive(35, 100, 100, 100)
    drive_distance_cm(35, FORWARD, 70, 73)

def Augmented_Reality_Statue():
    db.gyroRotation(10, 50, 50, 50)
    db.gyroStraightDrive(30, 50, 70, 50)
    db.gyroRotation(35, 50, 50, 50)
    drive_distance_cm(25, FORWARD, 50, 50)
    db.gyroStraightDrive(6, -50, -50, -50)
    db.gyroRotation(42, 50, 50, 50)
    db.gyroStraightDrive(40, 50, 100, 50)
    drive_distance_cm(24, FORWARD, 70, 70)
    db.gyroStraightDrive(9, -50, -50, -50)
    db.gyroRotation(20, 50, 50, 50, stopMethod=SM.stopResistance(db, 1500))
    db.gyroStraightDrive(8, -60, -60, -60)
    db.gyroRotation(45, 60, 60, 60)
    db.gyroStraightDrive(8, 40, 40, 40)
    drive_distance_cm(11, FORWARD, 30, 80)
    
def Dragon_Delivery():
    gyro_move(1.5, 80, FORWARD)
    wait_for_seconds(0.5)
    drive_distance_cm(42, BACKWARD, 80, 80)
    
def Boat_Cam__Rolling_Cam_Lever():
    drive_distance_cm(53, FORWARD, 62, 60)
    arm_down(210, 30)
    drive_distance_cm(12, BACKWARD, 40, 40)
    gyro_turn(30, CENTER, LEFT, 20)
    arm_up(40, 5)
    arm_up(170, 50)
    drive_distance_cm(35, BACKWARD, 50, 90)

#Runs
def run_1(vm, stack):
    wait_for_seconds(0.5)
    Pick_Up_Skateboard_Expert_New()
    Chicken_Printer_New()
    Collet_VE_Director_New()
    Tower_New()

def run_2(vm, stack):
    wait_for_seconds(0.5)
    Sound_Mixer_New()

def run_3(vm, stack):
    wait_for_seconds(0.5)
    Spinning_Show__Pick_Up_Expert_New()
    Deliver_Skateboard_Expert_New()

def run_4(vm, stack):
    wait_for_seconds(0.5)
    Dragon_Delivery()

def run_5(vm,stack):
    wait_for_seconds(0.5)
    Boat_Cam__Rolling_Cam_Lever()

def run_6(vm, stack):
    wait_for_seconds(0.5)
    Art_Piece_Delivery_New()
    Immersive_Experience_New()
    Stage_Push_Lights_New()
    
def run_7(vm, stack):
    wait_for_seconds(0.5)
    Rollling_Cam()
    Augmented_Reality_Statue()

def run_8(vm, stack):
     Augmented_Reality_Statue()

def Warm_up(vm, stack):
    db.gyroStraightDrive(10, 100, 100, 100)
    db.gyroStraightDrive(10, -30, -30, -30)
    db.gyroRotation(45, 50, 50, 50)
    db.gyroRotation(-90, 50, 50, 50)
    db.arcRotation(10, 30, 50, 50, 50)
    
# Battery Info
    
def show_battery_level():
    if hub.battery.capacity_left() < 81:
        hub.led(8)
    else:
        hub.led(6)

#Main Menu Initiation
run_funcs = [None, run_1, run_2, run_3, run_4, run_5, run_6, run_7, E, T, B, Warm_up]

run_names = [None, 1, 2, 3, 4, 5, 6, 7, 'E', 'T', 'B', 'W']

def update_config():
    global CONFIG
    CONFIG += 1
    if CONFIG == len(run_funcs):
        CONFIG = 1
    phub.light_matrix.write(run_names[CONFIG])

# Called on RIGHT button pressed
# Moves to next run
async def on_right_button(vm, stack):
    update_config()

# Called on LEFT button pressed
# Runs the run displayed, after run, moves to next run and waits for button press
async def on_left_button(vm, stack):
    global RUNNING
    if not RUNNING:
        RUNNING = 1
        before = time.get_time()
        run_funcs[CONFIG](vm, stack)
        after = time.get_time()
        print("Run {0} Time start={1}, end={2}, diff={3}".format(CONFIG, before, after, after - before))
        RUNNING = 0
        update_config()
    show_battery_level()

# Main Setup
async def on_start(vm, stack):
    phub.light_matrix.write(str(CONFIG))
    show_battery_level()


def setup(rpc, system, stop):
    print( hub.battery.capacity_left())
    vm = VirtualMachine(rpc, system, stop, "FLL2022")
    vm.register_on_button("on_left_button", on_left_button, "left", "pressed")
    vm.register_on_button("on_right_button", on_right_button, "right", "pressed")
    motorC.set_default_speed(DEFAULT_SPEED)
    motorD.set_default_speed(DEFAULT_SPEED)
    motorC.set_stop_action('brake')
    motorD.set_stop_action('coast')
    motorC.stop()
    motorD.stop()
    wheels.set_stop_action('coast')
    motorC.set_degrees_counted(0)
    motorD.set_degrees_counted(0)
    vm.register_on_start("another_unique_string", on_start)
    return vm

setup(None, system.system, sys.exit).start()
