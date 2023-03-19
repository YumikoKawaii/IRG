from matplotlib import pyplot as plt
import math
import numpy as np
import xlsxwriter

class Robot:
    #r is radius of wheels
    #R is radius of the chasis, only use for simulation purpose
    def __init__(self, r, L, R):
        self.r = r
        self.L = L
        self.R = R
    
    def setPosition(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta
        self.tracker = np.matrix([0, 0, px, py, theta], dtype=float)
    
    def setVelocity(self, vw, phi):
        self.v = vw*self.r/2
        self.w = self.v*math.tan(phi)/self.L
        
    def setDeltaT(self, dT):
        self.dT = dT

    def appendTracker(self):
        self.tracker = np.append(self.tracker,[[self.w, self.phi, self.px, self.py, self.theta]], axis=0)
    
    def nextPos(self):
        self.px = self.px + self.v*self.dT*math.cos((self.theta + self.theta + self.w*self.dT)/2)
        self.py = self.py + self.v*self.dT*math.sin((self.theta + self.theta + self.w*self.dT)/2)
        self.theta = self.theta + self.w*self.dT
        self.appendTracker()

