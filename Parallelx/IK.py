import numpy as np
import matplotlib.pyplot as plt
import serial as Serial
import time,warnings

def SolveIK(P,A,sign):
    dx, dy = P - A
    D = np.hypot(dx, dy)
    if not (abs(a - b) <= D <= a + b):
        return None
    phi = np.degrees(np.arctan2(dy, dx))
    alpha = np.degrees(np.arccos((a * a + D * D - b * b) / (2 * a * D)))
    return phi + sign * alpha

#Env setup.
a,b = 60, 70
origin = np.array([0, 0])
A1 = np.array([-12.9,0])
A2 = np.array([+12.9,0])
P = np.array([50,70])

#Solving.
teta1 = SolveIK(P,A1,1)
teta2 = SolveIK(P,A2,-1)
print("To go to",P)
print("Servo 1 & Servo 2 needs to move : ")
print(np.round(teta1,2),np.round(teta2,2))