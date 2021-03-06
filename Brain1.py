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
        self.angle_buffer = []

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
            self.traffic_light = []
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
                min_weight = 10 * INF
                min_angle = 0
                self.previous_count = self.count
                for angle in range(0, 360):
                    
                    if self.lidarThetaToGeneralTheta(angle) % 45 > 0:
                        continue
                    weight = self.astarweight(angle, trophy_point)
                    # print(angle, weight)
                    if min_weight > weight and weight < INF:
                        min_weight = weight                        
                        min_angle = angle
                    
                    if len(self.angle_buffer) > 5:
                        self.angle_buffer.pop(0)
                    self.angle_buffer.append(min_angle)
                
                min_angle = max(self.angle_buffer, key=self.angle_buffer.count)
                print(min_angle)
                self.goal = self.getPointByThetaFlip(self.lidarThetaToGeneralTheta(min_angle), r=30)
                
                self.goal_angle = min_angle
                self.goal_generic_angle = self.lidarThetaToGeneralTheta(self.goal_angle)
                # print(self.goal_generic_angle, min_weight)
                
                self.reinit = False
            
            if not self.reinit:
                self.reinitIfRespawn()
            
            
            self.controlVelocity()
            # self.controlAngle()
            
            if self.database.car.direction == self.goal_generic_angle:
                continue

            if (self.database.car.direction - self.goal_generic_angle) % 360 > 180 and self.database.lidar.data[180] > 30:
                self.left(5)
            elif self.database.lidar.data[0] > 30:
                self.right(5)
            

            

            # Implement Your Algorithm HERE!!

            # EXAMPLE CODE1: ?????? 3?????? ??????????????? ??????????????? ????????????

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
        point = self.getPointByThetaFlip(self.lidarThetaToGeneralTheta(theta), r=30)
        return self.getGlobalWeight(point, trophy_point) + self.getLocalWeight(theta)

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
        #         return INF
        return distance

    def getLocalWeight(self, theta):
        min_distance = 999
        for i in range(theta - 20, theta + 20):
            min_distance = min(min_distance, self.database.lidar.data[i % 360])
            
        if min_distance>=100:
            return 100 - min_distance
        else:
            return INF

    def controlVelocity(self):
        # if lidar[90] < 100 speed will go down.
        # if self.database.car.speed > MAX_SPEED -> self.down()

        min_distance = 999
        MAX_SPEED = 7
        for i in range(80, 110):
            min_distance = min(min_distance, self.database.lidar.data[i])

        if min_distance < 100:
            num= 10 - min_distance//10
            MAX_SPEED = 5 - 0.5 * num
            if self.database.car.speed > MAX_SPEED:
                if self.database.car.speed > 0:
                    self.down()
                if self.database.car.speed > 5:
                    self.down(2)
            else:
                self.up()
        else:       
            self.up()

        if self.isFacedTraffic() and self.database.car.speed > 0:
            self.down(self.database.car.speed)

    
    def getPointByTheta(self, theta, r=100):
        
        x = self.car_point[0]+r*math.cos(self.toRadian(theta))
        y = self.car_point[1]-r*math.sin(self.toRadian(theta))

        return (x, y)
    
    def getPointByThetaFlip(self, theta, r=100):
        x = self.car_point[0]-r*math.sin(self.toRadian(theta))
        y = self.car_point[1]-r*math.cos(self.toRadian(theta))
        # if printable:
        #     print(r)
        return (x if x > 0 else 0, y if y > 0 else 0)

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

        # ????????????,
        # ???????????? ???????????? ?????????
        # ????????? ??????????????? ?????? ?????????
        # ????????????.

        Closeness=False
        runout=False
        faced = False

        for traffic in self.traffic_light:
            traffic_position = traffic['position']

            if distance(self.database.car.position, traffic_position)< 100:
                Closeness = True
            else:
                return False

            remain_time=traffic["remain_time"]

            if remain_time < 2: 
                runout = True

            if traffic['light'] == 'green' and not runout:
                return False

            if traffic['light'] == 'red' and runout:
                return False
            

            x = self.car_point[0]
            y = self.car_point[1]
            cos = math.cos(self.goal_generic_angle)
            sin = math.sin(self.goal_generic_angle)

            for i in range(100):
                x = x + sin
                y = y + cos
                
                
                width = traffic['width']
                height = traffic['height']
                if traffic_position[0] - width / 2 <= x <= traffic_position[0] + width / 2 and \
                   traffic_position[1] - height / 2 <= y <= traffic_position[1] + height / 2:
                   faced = True

            
            
            
            if Closeness and faced:
                if traffic['light'] == 'red':
                    return True
        return False
