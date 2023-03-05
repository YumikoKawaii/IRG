from matplotlib import pyplot as plt
import math
import numpy as np
import xlsxwriter

#robot class
class Robot:

    #R is radius of chassis
    def __init__(self, r, l, R):
        self.r = r
        self.l = l
        self.R = R
    
    def setPosition(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta
        self.tracker = np.matrix([0, 0, 0, px, py, theta], dtype=float)
    
    def setVelocity(self, w1, w2, w3):

        self.w1 = w1
        self.w2 = w2
        self.w3 = w3

        vw = np.matrix([[w1],[w2],[w3]])

        rMatrix = np.matrix([[math.sin(math.pi/3), math.sin(math.pi), math.sin(5*math.pi/3)],
                             [-math.cos(math.pi/3), -math.cos(math.pi), -math.cos(5*math.pi/3)],
                             [-1/self.l, -1/self.l, -1/self.l]])

        result = (self.r/3)*rMatrix*vw
        
        self.vx = result[0, 0]
        self.vy = result[1, 0]
        self.w = result[2, 0]

    def setDeltaT(self, dT):
        self.dT = dT

    def appendTracker(self):
        self.tracker = np.append(self.tracker,[[self.w1, self.w2, self.w3, self.px, self.py, self.theta]], axis=0)
    
    def nextPos(self):
        
        angle = (self.theta + self.theta + self.w*self.dT)/2

        self.px += self.vx*self.dT*math.cos(angle) + self.vy*self.dT*math.cos(angle + math.pi/2)
        self.py += self.vx*self.dT*math.sin(angle) + self.vy*self.dT*math.sin(angle + math.pi/2)
        self.theta += self.w*self.dT

        self.appendTracker()

#set initial state
robot = Robot(0.05, 0.1, 0.12)
robot.setPosition(0, -2, 0)
robot.setDeltaT(0.01)

#create moving plan
plan = np.matrix([0, 0, 0, 0], dtype=float)
plan = np.append(plan, [[4, 8*math.pi, -1*math.pi, -8*math.pi]], axis=0)
plan = np.append(plan, [[3, 12*math.pi, -2*math.pi, -12*math.pi]], axis=0)
plan = np.append(plan, [[2, 16*math.pi, -4*math.pi, -16*math.pi]], axis=0)
plan = np.append(plan, [[1, 20*math.pi, -8*math.pi, -20*math.pi]], axis=0)
plan = np.append(plan, [[1, 16*math.pi, 0, -16*math.pi]], axis=0)
plan = np.append(plan, [[1, 8*math.pi, 8*math.pi, -16*math.pi]], axis=0)

phi = np.arange(0,2*np.pi,0.1)
x_circle = robot.R*np.cos(phi)
y_circle = robot.R*np.sin(phi)

#moving function
def move(t, w1, w2, w3):

    robot.setVelocity(w1, w2, w3)

    while (t > 0):
        t -= robot.dT
        robot.nextPos()

        #chassis border
        x_r = robot.px + x_circle
        y_r = robot.py + y_circle

        #heading
        x2 = robot.px + (robot.R + 0.05)*math.cos(robot.theta)
        y2 = robot.py + (robot.R + 0.05)*math.sin(robot.theta)

        path, = plt.plot(robot.tracker[0:len(robot.tracker), 3], robot.tracker[0:len(robot.tracker), 4], color="red")
        line, = plt.plot([robot.px, x2], [robot.py, y2],color="black")
        r, = plt.plot(x_r, y_r, color="black")
        
        plt.xlim(-2.5,2.5)
        plt.ylim(-2.5,2.5)
        plt.gca().set_aspect('equal')
        plt.pause(0.000001)
        r.remove()
        line.remove()
        if t - 0 > 0.00000001:
            path.remove()

def perform(plan):

    for i in plan:
        move(i[0,0], i[0,1], i[0,2], i[0, 3])

perform(plan)

workbook = xlsxwriter.Workbook("position_trackings.xlsx")
worksheet = workbook.add_worksheet()

worksheet.write('A1',"w_1")
worksheet.write('B1',"w_2")
worksheet.write('C1',"w_3")
worksheet.write('D1',"pos_x")
worksheet.write('E1',"pos_y")
worksheet.write('F1',"theta")

index = 2

for i in robot.tracker:
    worksheet.write("A{}".format(index),i[0, 0])
    worksheet.write("B{}".format(index),i[0, 1])
    worksheet.write("C{}".format(index),i[0, 2])
    worksheet.write("D{}".format(index),i[0, 3])
    worksheet.write("E{}".format(index),i[0, 4])
    worksheet.write("F{}".format(index),i[0, 5])
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