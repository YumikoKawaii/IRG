from matplotlib import pyplot as plt
import math
import numpy as np
import xlsxwriter

#robot class
class Robot:
    #L is the length of chasis
    #r is the radis of wheels
    #d is the radius of chasis, only use for simulation
    def __init__(self, r, L, d):
        self.r = r
        self.L = L
        self.d = d
    
    def setPosition(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta
        self.tracker = np.matrix([0, 0, 0, px, py, theta], dtype=float)
    
    #vw is wheel's speed
    #phi is angle between hub of front-wheel and hub of rear wheel(?)
    #R is the radius of the trajectory 
    def setVelocity(self, vw, phi):
        self.R = 0 if math.tan(phi) == 0 else abs(self.L/math.tan(phi))
        self.v = vw*self.r/2
        self.w = self.v*math.tan(phi)/self.L
        
    def setDeltaT(self, dT):
        self.dT = dT

    def appendTracker(self):
        self.tracker = np.append(self.tracker,[[self.v, self.w, self.R, self.px, self.py, self.theta]], axis=0)
    
    def nextPos(self):
        self.px = self.px + self.v*self.dT*math.cos((self.theta + self.theta + self.w*self.dT)/2)
        self.py = self.py + self.v*self.dT*math.sin((self.theta + self.theta + self.w*self.dT)/2)
        self.theta = self.theta + self.w*self.dT
        self.appendTracker()

#set initial state
robot = Robot(0.05, 0.2, 0.12)
robot.setPosition(-1, -1, math.pi/3)
robot.setDeltaT(0.01)

#create moving plan
plan = np.matrix([0, 0, 0], dtype=float)
plan = np.append(plan, [[1, 8*math.pi, 0]], axis=0)
plan = np.append(plan, [[1, 8*math.pi, -math.pi/6]], axis=0)
plan = np.append(plan, [[1, 8*math.pi, math.pi/6]], axis=0)
plan = np.append(plan, [[1, 8*math.pi, -math.pi/6]], axis=0)
plan = np.append(plan, [[1, 8*math.pi, math.pi/6]], axis=0)

phi = np.arange(0,2*np.pi,0.1)
x_circle = robot.d*np.cos(phi)
y_circle = robot.d*np.sin(phi)

#moving function
def move(t, vw, phi):

    robot.setVelocity(vw, phi)

    while (t > 0):
        t -= robot.dT
        robot.nextPos()

        #chasis border
        x_r = robot.px + x_circle
        y_r = robot.py + y_circle

        #heading
        x2 = robot.px + (robot.d + 0.05)*math.cos(robot.theta)
        y2 = robot.py + (robot.d + 0.05)*math.sin(robot.theta)

        path, = plt.plot(robot.tracker[0:len(robot.tracker), 3], robot.tracker[0:len(robot.tracker), 4], color="red")
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

worksheet.write('A1',"v")
worksheet.write('B1',"w")
worksheet.write('C1',"R")
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
    x2 = robot.px + (robot.d + 0.05)*math.cos(robot.theta)
    y2 = robot.py + (robot.d + 0.05)*math.sin(robot.theta)

    plt.plot([robot.px, x2], [robot.py, y2],color="black")
    plt.plot(x_r, y_r, color="black")

lastPos()
plt.show()