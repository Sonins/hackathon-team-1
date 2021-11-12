import time
import pygame
import math
from typing import Iterable, Tuple
import matplotlib.pyplot as plt

from LiDAR import LiDAR

INF = 10000000

def distance(point1: Tuple, point2: Tuple):
    xdiff = point1[0] - point2[0]
    ydiff = point1[1] - point2[1]
    return math.sqrt(xdiff * xdiff + ydiff * ydiff)


class Brain1:
    def __init__(self, database):
        self.database = database
        self.map = [[0] * 2000] * 1300
        self.goal_angle = 0
        self.reinit = False
        self.count = 10
        self.previous_count = -999
        self.traffic_light = []
        self.car_point = None

    def run(self):
        self.goal = self.database.car.position
        
        while True:
            if self.database.stop:
                break

            time.sleep(0.001)
            _ = pygame.event.get()

            if not self.database.v2x_data or not self.database.lidar.data:
                continue

            

            trophy_point = self.database.v2x_data['Trophy']
            
            for i in self.database.v2x_data.values():
                if i[0] == 'Crosswalk':
                    traffic = {}
                    traffic['light'] = i[1]
                    traffic['position'] = i[2]
                    traffic['height'] = i[3]
                    traffic['width'] = i[4]
                    traffic['remain_time'] = i[5]
                    self.traffic_light.append(traffic)

            self.car_point = self.database.car.position
            self.count = self.count + 1
            # self.database.car.last_collision
            for i in range(0, 360):
                if self.database.lidar.data[i] == 100:
                    continue
                point = self.getPointByThetaFlip(self.lidarThetaToGeneralTheta(i), self.database.lidar.data[i])
                self.map[0 if round(point[0]) < 0 else round(point[0])][0 if round(point[1]) < 0 else round(point[1])] = 1

            if self.count - self.previous_count > 20:
                min_weight = INF * 2
                min_angle = 0
                self.previous_count = self.count
                for angle in range(0, 360, 45):
                    weight = self.astarweight(angle, trophy_point)
                    if min_weight > weight and weight < INF:
                        min_weight = weight                        
                        min_angle = angle
                print(min_angle, min_weight, self.database.lidar.data[min_angle])
                self.goal = self.getPointByThetaFlip(self.lidarThetaToGeneralTheta(min_angle), r=30)
                
                self.goal_angle = min_angle
                self.goal_generic_angle = self.lidarThetaToGeneralTheta(self.goal_angle)
                
                self.reinit = False
            
            if not self.reinit:
                self.reinitIfRespawn()
            
            
            self.controlVelocity()
            # self.controlAngle()
            
            if self.database.car.direction == self.goal_generic_angle:
                continue

            if (self.database.car.direction - self.goal_generic_angle) % 360 > 180:
                self.left(5)
            else:
                self.right(5)
            

            

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

    def astarweight(self, theta, trophy_point):
        point = self.getPointByThetaFlip(self.lidarThetaToGeneralTheta(theta))
        return self.getGlobalWeight(point, trophy_point) + self.getLocalWeight(theta)
        # return self.getLocalWeight(self.car_point, theta)

    def getGlobalWeight(self, point: Tuple, trophy_point: Tuple):
        xdiff = trophy_point[0] - point[0]
        ydiff = trophy_point[1] - point[1]
        distance = math.sqrt(xdiff * xdiff + ydiff * ydiff)

        sin = ydiff / distance
        cos = xdiff / distance
        
        x = point[0]
        y = point[1]

        # while abs(trophy_point[0] - x) > 1:
        #     x = x + cos
        #     y = y + sin
        #     # if there is wall
        #     if self.map[round(x)][round(y)] == 1:
        #         return 2 * (distance)
        return distance / 10

    def getLocalWeight(self, theta):
        min_distance = 999
        for i in range(theta - 22, theta + 23):
            min_distance = min(min_distance, self.database.lidar.data[i % 360])
            
        if min_distance>=50:
            return 100 - min_distance
        else:
            return INF * (100 - min_distance)

    def controlVelocity(self):
        # if lidar[90] < 100 speed will go down.
        # if self.database.car.speed > MAX_SPEED -> self.down()

        min_distance = 999
        MAX_SPEED = 7
        for i in range(80, 110):
            min_distance = min(min_distance, self.database.lidar.data[i])

        if min_distance < 100:
            num= 10 - min_distance//10
            MAX_SPEED = 7 - 0.7 * num
            if self.database.car.speed > MAX_SPEED:
                self.down()
            else:
                self.up()
        else:       
            self.up()

        if self.isFacedTraffic() and self.database.car.speed > 0:
            self.down()

    
    def getPointByTheta(self, theta, r=100):
        
        x = self.car_point[0]+r*math.cos(self.toRadian(theta))
        y = self.car_point[1]-r*math.sin(self.toRadian(theta))

        return (x, y)
    
    def getPointByThetaFlip(self, theta, r=100):
        x = self.car_point[0]-r*math.sin(self.toRadian(theta))
        y = self.car_point[1]-r*math.cos(self.toRadian(theta))
        # if printable:
        #     print(r)
        return (x, y)

    def toRadian(self, theta):
        return theta * (math.pi / 180.0)

    def isArriveAtGoal(self):
        if distance(self.goal, self.car_point) < 5:
            return True
        return False            
        
        
    def controlAngle(self):
        if self.database.car.direction>0 and self.database.car.direction<=90:
            if self.goal_generic_angle>self.database.car.direction and self.goal_generic_angle<180+self.database.car.direction:
                self.left()
            else:
                self.right()

        elif self.database.car.direction>90 and self.database.car.direction<=180:
            if self.goal_generic_angle<self.database.car.direction or (540-self.database.car.direction)<self.goal_generic_angle:
                self.right()
            else:
                self.left()

        elif self.database.car.direction>180 and self.database.car.direction<=270:
            if self.goal_generic_angle<self.database.car.direction or (540-self.database.car.direction)<self.goal_generic_angle:
                self.left()
            else:
                self.right()
                
        elif self.database.car.direction>270 and self.database.car.direction<=360:
            if self.goal_generic_angle>self.database.car.direction and self.goal_generic_angle<180+self.database.car.direction:
                self.right()
            else:
                self.left()


    def reinitIfRespawn(self):
        if self.database.car.last_collision < 999:
            self.reinit = True

    def lidarThetaToGeneralTheta(self, theta):
        return (theta + self.database.car.direction - 90) % 360

    def isFacedTraffic(self):

        # 가까울때,
        # 신호등을 마주보고 있을때
        # 신호등 남은시간이 얼마 없을때
        # 멈춰야함.

        Closeness=False
        runout=False
        faced = False

        for traffic in self.traffic_light:
            x = self.car_point[0]
            y = self.car_point[1]
            cos = math.cos(self.goal_generic_angle)
            sin = math.sin(self.goal_generic_angle)

            for i in range(30):
                x = x + sin
                y = y + cos
                
                traffic_position = traffic['position']
                width = traffic['width']
                height = traffic['height']
                if traffic_position[0] - width / 2 <= x <= traffic_position[0] + width / 2 and \
                   traffic_position[1] - height / 2 <= y <= traffic_position[1] + height / 2:
<<<<<<< HEAD
                   faced = True

            if distance(self.database.car.position, traffic_position)< 5:
                Closeness = True

            remain_time=traffic["remain_time"]
            if remain_time<2: 
                runout = True
            if runout and Closeness and faced:
                return True
        return False
=======
                   return True
        return False
        
>>>>>>> 2f6af7429fef2987b57574680c13f2e0d5c6c104
