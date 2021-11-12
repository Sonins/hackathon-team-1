import time
import pygame
import math
from typing import Iterable, Tuple
import matplotlib.pyplot as plt

from LiDAR import LiDAR

INF = 1000000000

def distance(point1: Tuple, point2: Tuple):
    xdiff = point1[0] - point2[0]
    ydiff = point1[1] - point2[1]
    return math.sqrt(xdiff * xdiff + ydiff * ydiff)


class Brain1:
    def __init__(self, database):
        self.database = database
        self.map = [[0] * 300] * 400
        self.goal_angle = 0
        self.reinit = False

    def run(self):
        self.goal = self.database.car.position
        
        while True:
            if self.database.stop:
                break

            time.sleep(0.001)
            _ = pygame.event.get()


            '''
            ☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
            ☆☆☆☆☆ DO NOT CHANGE ANOTHER CODE IN 2021-Hackathon-Simulator!!! ☆☆☆☆☆
            ☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆ ONLY CHANGE Brain.py!!!☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
            ☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆

            1. How can I get a lidar / gps / imu data?
                Lidar : data = self.database.lidar.data
                Gps : data = self.database.car.position
                IMU : data = self.database.car.direction

            2. How can I move a car?
                self.database.control.up()
                self.database.control.down()
                self.database.control.right()
                self.database.control.left()

                OR

                self.up(num)
                self.down(num)
                self.right(num)
                self.left(num)
                ☆☆☆☆☆ num in here is number of acceleration ☆☆☆☆☆

                ☆☆☆☆☆
                In one loop,
                you can only change the acceleration up to 5 and the angle up to 8!!
                Maximum speed of car is 15 and maximum angle of car can rotate is 8!!
                ☆☆☆☆☆

            3. How can I get a car status data?
                self.database.car.direction
                self.database.car.speed

            4. How can I get a v2x data?
                self.database.v2x_data
            '''


            if not self.database.v2x_data or not self.database.lidar.data:
                continue

            

            trophy_point = self.database.v2x_data['Trophy']
            car_point = self.database.car.position
            # self.database.car.last_collision

            for i in range(0, 360):
                if self.database.lidar.data[i] == 100:
                    continue
                if self.lidarThetaToGeneralTheta(i) == 0:
                    print(point, i)
                    point = self.getPointByThetaFilp(car_point, self.lidarThetaToGeneralTheta(i), self.database.lidar.data[i], True)
                else:
                    point = self.getPointByThetaFilp(car_point, self.lidarThetaToGeneralTheta(i), self.database.lidar.data[i])
                
                self.map[round(point[0])][round(point[1])] = 1

            if self.isArriveAtGoal(car_point) or self.reinit:
                min_weight = INF * 2
                min_angle = 0
                for angle in range(0, 360, 45):
                    weight = self.astarweight(car_point, angle, trophy_point)
                    if min_weight > weight:
                        min_weight = weight                        
                        min_angle = angle
                
                self.goal = self.getPointByThetaFilp(car_point, self.lidarThetaToGeneralTheta(min_angle), r=10)
                # print(f"{car_point}-->{self.goal}, angle: {self.lidarThetaToGeneralTheta(min_angle)}")
                self.goal_angle = min_angle
                self.goal_generic_angle = self.lidarThetaToGeneralTheta(self.goal_angle)
                self.reinit = False

            if not self.reinit:
                self.reinitIfRespawn()
            
            
            self.controlVelocity()
            self.controlAngle()
            

            # Implement Your Algorithm HERE!!

            # EXAMPLE CODE1: 속도 3으로 유지하면서 오른쪽으로 회전하기

    def up(self, num: int = 1):
        for i in range(num):
            self.database.control.up()

    def down(self, num: int = 1):
        for i in range(num):
            self.database.control.down()

    def right(self, num: int = 1):
        for i in range(num):
            self.database.control.right()

    def left(self, num: int = 1):
        for i in range(num):
            self.database.control.left()

    def astarweight(self, car_point, theta, trophy_point):
        x = car_point[0]+self.database.lidar.data[theta]*math.cos(theta)
        y = car_point[1]+self.database.lidar.data[theta]*math.sin(theta)
        point = (x, y)
        return self.getGlobalWeight(point, trophy_point) + self.getLocalWeight(car_point, theta)

    def getGlobalWeight(self, point: Tuple, trophy_point: Tuple):
        xdiff = trophy_point[0] - point[0]
        ydiff = trophy_point[1] - point[1]
        distance = math.sqrt(xdiff * xdiff + ydiff * ydiff)

        sin = ydiff / distance
        cos = xdiff / distance
        
        x = point[0]
        y = point[1]

        while abs(trophy_point[0] - x) < 1:
            x = x + cos
            y = y + sin
            # if there is wall
            if self.map[round(x)][round(y)] == 1:
                return INF
        
        return distance

    def getLocalWeight(self, car_point, theta):
        if self.database.lidar.data[theta]>=100:
            return 100
        elif self.database.lidar.data[theta]<100:
            return INF

    def controlVelocity(self):
        # if lidar[90] < 100 speed will go down.
        # if self.database.car.speed > MAX_SPEED -> self.down()

        min_distance = 999
        for i in range(75, 105):
           min_distance = min(min_distance, self.database.lidar.data[i])

        if min_distance < 100:
            num= 10 - min_distance//10
            MAX_SPEED = 10 - 0.7 * num
            if self.database.car.speed > MAX_SPEED:
                self.down()
            else:
                self.up()
        else:
            self.up()

    
    def getPointByTheta(self, car_point, theta, r=100):
        
        x = car_point[0]+r*math.cos(self.toRadian(theta))
        y = car_point[1]-r*math.sin(self.toRadian(theta))

        return (x, y)
    
    def getPointByThetaFilp(self, car_point, theta, r=100, printable=False):
        x = car_point[0]+r*math.sin(self.toRadian(theta))
        y = car_point[1]-r*math.cos(self.toRadian(theta))
        if printable:
            print(r)
        return (x, y)

    def toRadian(self, theta):
        return theta * (math.pi / 180.0)

    def isArriveAtGoal(self, car_point):
        if distance(self.goal, car_point) < 5:
            return True
        return False            
        
        
    def controlAngle(self):
        if self.database.car.direction>0 and self.database.car.direction<90:
            if self.goal_generic_angle>self.database.car.direction and self.goal_generic_angle<180+self.goal_generic_angle:
                self.left()
            else:
                self.right()

        elif self.database.car.direction>90 and self.database.car.direction<180:
            if self.goal_generic_angle<self.database.car.direction or (540-self.database.car.direction)<self.goal_generic_angle:
                self.right()
            else:
                self.left()

        elif self.database.car.direction>180 and self.database.car.direction<270:
            if self.goal_generic_angle<self.database.car.direction or (540-self.database.car.direction)<self.goal_generic_angle:
                self.left()
            else:
                self.right()
                
        elif self.database.car.direction>270 and self.database.car.direction<360:
            if self.goal_generic_angle>self.database.car.direction and self.goal_generic_angle<180+self.goal_generic_angle:
                self.right()
            else:
                self.left()


    def reinitIfRespawn(self):
        if self.database.car.last_collision < 999:
            self.reinit = True

    def lidarThetaToGeneralTheta(self, theta):
        return (theta + self.database.car.direction - 90) % 360
