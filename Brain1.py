import time
import pygame
import math
from typing import Iterable, Tuple
# import matplotlib.pyplot

from LiDAR import LiDAR

INF = 1000000000

def distance(point1: Tuple, point2: Tuple):
    xdiff = point1[0] - point2[0]
    ydiff = point1[1] - point2[1]
    return math.sqrt(xdiff * xdiff + ydiff * ydiff)


class Brain1:
    def __init__(self, database):
        self.database = database
        self.map = [[0] * 1000] * 2000
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
                point = self.getPointByTheta(car_point, self.lidarThetaToGeneralTheta(i), self.database.lidar.data[i])
                self.map[round(point[0])][round(point[1])] = 1
                

            if self.isArriveAtGoal(car_point) or self.reinit:
                min_weight = INF * 2
                min_angle = 0
                for angle in range(0, 360, 45):
                    weight = self.astarweight(car_point, angle, trophy_point)
                    if min_weight > weight:
                        min_weight = weight
                        min_angle = angle

                self.goal = self.getPointByTheta(car_point, min_angle, r=10)
                self.goal_angle = min_angle
                self.reinit = False
                print(self.goal_angle)
            
            
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
        if self.database.lidar.data[90] < 100:
            num= 10 - self.database.lidar.data[90]//10
            MAX_SPEED = 10 - 0.7 * num
            if self.database.car.speed > MAX_SPEED:
                self.down()
            else:
                self.up()
        else:
            self.up()

    
    def getPointByTheta(self, car_point, theta, r=100):
        
        x = car_point[0]+r*math.cos(self.toRadian(theta))
        y = car_point[1]+r*math.sin(self.toRadian(theta))

        return (x, y)
    
    def getPointByThetaFilp(self, car_point, theta, r=100):
        x = car_point[0]+r*math.sin(self.toRadian(theta))
        y = car_point[1]+r*math.cos(self.toRadian(theta))

        return (x, y)

    def toRadian(self, theta):
        return theta * (math.pi / 180.0)

    def isArriveAtGoal(self, car_point):
        if distance(self.goal, car_point) < 5:
            return True
        return False            
        
        
    def controlAngle(self):
        if (self.goal_angle>0 and self.goal_angle<90) or (self.goal_angle >= 270 and self.goal_angle < 360):
            self.right()
        elif self.goal_angle > 90 and self.goal_angle < 270:
            self.left()
            
    def reinitIfRespawn(self):
        pass

    def lidarThetaToGeneralTheta(self, theta):
        return (theta + self.database.car.direction - 90) % 360
