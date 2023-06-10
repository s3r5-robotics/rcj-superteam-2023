import math
import time

import cv2 as cv
import numpy as np
import numpy.typing as npt

from controller import Robot, Motor, Camera
from distance import DistanceCalculator

robot: Robot = Robot()
timestep = int(robot.getBasicTimeStep()  / 4)

wheel_left: Motor = robot.getDevice("left motor")
left_encoder = wheel_left.getPositionSensor()
wheel_right: Motor = robot.getDevice("right motor")
right_encoder = wheel_right.getPositionSensor()
left_encoder.enable(timestep)
right_encoder.enable(timestep)

wheel_left.setPosition(float('inf'))  # Make sure the motors will not stop on their own
wheel_right.setPosition(float('inf'))

# distance_sensor: DistanceSensor = robot.getDevice("distance_sensor")
# distance_sensor.enable(timestep)

camera: Camera = robot.getDevice("camera")
camera.enable(timestep)
camera_height: int = camera.getHeight()
camera_width: int = camera.getWidth()
distance_calculator: DistanceCalculator = DistanceCalculator(camera_height, camera_width)


def camera_get_image():
    image_data = camera.getImage()
    return np.array(np.frombuffer(image_data, np.uint8).reshape((camera_height, camera_width, 4)))


def camera_get_distance(image: npt.ArrayLike):
    return distance_calculator.distance_calculation(image)


def findHole(image: npt.ArrayLike):
    lower_bla = np.array([0, 0, 0])
    upper_bla = np.array([10, 10, 30])
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    edited = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    blackNum = np.sum(cv.inRange(edited, lower_bla, upper_bla))
    print('Jama', blackNum)
    if blackNum > 150000:
        return True
    else:
        return False


def findWall(image: npt.ArrayLike):
    lower_ste = np.array([80, 0, 0])
    upper_ste = np.array([120, 200, 200])
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    edited = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    wall = np.sum(cv.inRange(edited, lower_ste, upper_ste))
    print('Stena', wall)
    if wall > 650000:
        return True
    else:
        return False


max_velocity = 6.28
speeds = [max_velocity, max_velocity]


def delay(ms):
    initTime = robot.getTime()
    while robot.step(timestep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break

def spin_right():
    speeds[0] = max_velocity
    speeds[1] = -max_velocity


def spin_left():
    speeds[0] = -max_velocity
    speeds[1] = max_velocity


def pivot_angle(angle):
    count_per_rev = 20

    wheel_rad = 20.5 * 2
    wheel_circ = wheel_rad * math.pi

    pivot_diam = 14.95
    pivot_circ = math.pi * pivot_diam

    distance = pivot_circ * abs(angle) / 360
    numRev = distance / wheel_circ
    target_count = numRev * count_per_rev

    while robot.step(timestep) != -1:
        if angle > 0:
            value = abs(left_encoder.getValue())
        else:
            value = abs(right_encoder.getValue())

        print(f"{value} | {target_count}")

        if value >= target_count:
            break
        if angle > 0:
            spin_left()
        else:
            spin_right()
        wheel_left.setVelocity(speeds[0])
        wheel_right.setVelocity(speeds[1])


while robot.step(timestep) != -1:
    pivot_angle(-90)
    # wheel_left.setVelocity(max_velocity)
    # wheel_right.setVelocity(max_velocity)
    # delay(900)
    #
    # camera_image = camera_get_image()
    # ph = distance_calculator.pixel_height(camera_image)
    # print(f"ph {ph}; hole {findHole(camera_image)}; wall {findWall(camera_image)}")
    #
    # wheel_left.setVelocity(0)
    # wheel_right.setVelocity(0)
    time.sleep(2)
