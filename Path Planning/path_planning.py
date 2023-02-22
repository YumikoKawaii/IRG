from matplotlib import pyplot as plt
import math
import numpy as np
import xlsxwriter

#robot class
class Robot:
    #r is radius of wheels, R is radius of the chasis
    #l is distance between tracking point and wheels
    def __init__(self, r, l, R):
        self.r = r
        self.l = l
        self.R = R
    
    def setPosition(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta
        self.tracker = np.matrix([px, py, theta], dtype=float)
    
    def setVelocity(self, wl, wr):
        self.w = (wr - wl)*self.r/self.l
        self.v = (wr + wl)*self.r/2
        
    def setDeltaT(self, dT):
        self.dT = dT

    def appendTracker(self):
        self.tracker = np.append(self.tracker,[[self.px, self.py, self.theta]], axis=0)
    
    def nextPos(self):
        self.px = self.px + self.v*self.dT*math.cos((self.theta + self.theta + self.w*self.dT)/2)
        self.py = self.py + self.v*self.dT*math.sin((self.theta + self.theta + self.w*self.dT)/2)
        self.theta = self.theta + self.w*self.dT
        self.appendTracker()

#set initial state
robot = Robot(0.02, 0.05, 0.07)
robot.setPosition(0, -1, math.pi/3)
robot.setDeltaT(0.01)

#create moving plan
plan = np.matrix([0, 0, 0], dtype=float)
plan = np.append(plan, [[0.1, 50, 50]], axis=0)
plan = np.append(plan, [[1, 52, 50]], axis=0)
plan = np.append(plan, [[1.8, 50, 55]], axis=0)
plan = np.append(plan, [[0.3, 10, -10]], axis=0)
plan = np.append(plan, [[2.6, 50, 54]], axis=0)
plan = np.append(plan, [[2, 51, 50]], axis=0)

phi = np.arange(0,2*np.pi,0.1)
x_circle = robot.R*np.cos(phi)
y_circle = robot.R*np.sin(phi)

#moving function
def move(t, wl, wr):

    robot.setVelocity(wl, wr)

    while (t > 0):
        t -= robot.dT
        robot.nextPos()

        #chasis border
        x_r = robot.px + x_circle
        y_r = robot.py + y_circle

        #heading
        x2 = robot.px + (robot.R + 0.05)*math.cos(robot.theta)
        y2 = robot.py + (robot.R + 0.05)*math.sin(robot.theta)

        path, = plt.plot(robot.tracker[0:len(robot.tracker), 0], robot.tracker[0:len(robot.tracker), 1], color="red")
        line, = plt.plot([robot.px, x2], [robot.py, y2],color="black")
        r, = plt.plot(x_r, y_r, color="black")
        
        plt.xlim(-2,2)
        plt.ylim(-2,2)
        plt.gca().set_aspect('equal')
        plt.pause(0.000001)
        r.remove()
        line.remove()
        if t - 0 > 0.00000001:
            path.remove()


def perform(plan):

    for i in plan:
        move(i[0,0], i[0,1], i[0,2])
    
perform(plan)

workbook = xlsxwriter.Workbook("position_tracking.xlsx")
worksheet = workbook.add_worksheet()

worksheet.write('A1',"Pos_x")
worksheet.write('B1',"Pos_y")
worksheet.write('C1',"Theta")

index = 2

for i in robot.tracker:
    worksheet.write("A{}".format(index),i[0, 0])
    worksheet.write("B{}".format(index),i[0, 1])
    worksheet.write("C{}".format(index),i[0, 2])
    index+=1

workbook.close()

def lastPos():
    #chasis border
    x_r = robot.px + x_circle
    y_r = robot.py + y_circle

    #heading
    x2 = robot.px + (robot.R + 0.05)*math.cos(robot.theta)
    y2 = robot.py + (robot.R + 0.05)*math.sin(robot.theta)

    plt.plot([robot.px, x2], [robot.py, y2],color="black")
    plt.plot(x_r, y_r, color="black")

lastPos()
plt.show()