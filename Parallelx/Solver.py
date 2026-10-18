import numpy as np
import matplotlib.pyplot as plt

#constants
#origin is midway between the two servos.
origin = np.array([0, 0])
#distance between servos.
dlink = 25.8
#a short arm (servo to elbow), b long arm (elbow to pen).
a,b = 60, 70
#servo angles in degrees, ccw from +x.
teta1, teta2 = 120, 60

# find the A1,A2 Servo points
#fixed points, never move.
A1 = origin + np.array([-dlink / 2, 0])
A2 = origin + np.array([dlink / 2, 0])

#refer structure from src/structure.png
#FK: angles -> elbow B1,B2 and pen P.
def FK(teta1,teta2):
    #degree to radians to use np.cos,np.sin
    t1rad = np.radians(teta1)
    t2rad = np.radians(teta2)
    #Elbow point B1,B2.
    #start at servo, walk a=60 along the short arm direction (cos,sin).
    B1 = A1 + a * np.array([np.cos(t1rad), np.sin(t1rad)])
    B2 = A2 + a * np.array([np.cos(t2rad), np.sin(t2rad)])
    #Difference and Length between the points.
    #V elbow to elbow arrow, L its length.
    V = B2 - B1
    L = np.linalg.norm(V)
    #Physically not possible.
    #long arms are 70 each so elbows max 140 apart. L==0 breaks the divide in n.
    if(L>2*b or L==0) : return (None,None,None)
    #Take midpoint and calculate the h.
    #Project that point on the direction of the vector
    #that is perpendicular to the vector V.
    #B1,P,B2 isoceles triangle. pen is above midpoint M at height h, pythagoras.
    M = 0.5 * (B1 + B2)
    h = np.sqrt(b * b - (L / 2) ** 2)
    #n = V rotated 90 and made length 1. +h*n is the pen on paper side, -h*n other assembly.
    n = np.array([-V[1], V[0]]) / L
    #no rounding here, it spoils jacobian and loop check. round only when printing.
    return B1, B2, M + h * n

#From (x,y) to (teta1,teta2)
#one arm only. P pen, A that arm servo, sign +1 left arm -1 right arm.
def IK(P,A,sign):
    #arrow servo to pen and its length D.
    delx, dely = P - A
    D = np.hypot(delx, dely)
    #reach check. triangle a,b,D closes only if 10<=D<=130 else arccos gets value out of -1..1
    if not abs(a - b) <= D <= a + b:
        return None
    #phi direction servo to pen. atan2 so all quadrants ok.
    phi = np.degrees(np.arctan2(dely, delx))
    #alpha angle at servo between line A->P and short arm. law of cosines.
    alpha = np.degrees(np.arccos((a * a + D * D - b * b) / (2 * a * D)))
    #short arm sits alpha away from line A->P. + for left - for right gives elbow out.
    return phi + sign * alpha

#Pen velocity Vx,Vy for given omega. this is V = J*omega for one omega.
#omega in rad/s. returns V in mm/s and det of LHS.
def PenVelocity(teta1,teta2,omega1,omega2):
    B1,B2,P=FK(teta1,teta2)
    if(B1 is None) : return None
    #R1,R2 long arms as arrows elbow to pen.
    R1,R2=P-B1,P-B2
    t1rad=np.radians(teta1)
    t2rad=np.radians(teta2)
    #elbow velocity. elbow is on circle radius a around servo.
    #direction (-sin,cos) is short arm turned 90 (tangent). speed a*omega.
    U1 = a * omega1 * np.array([-np.sin(t1rad), np.cos(t1rad)])
    U2 = a * omega2 * np.array([-np.sin(t2rad), np.cos(t2rad)])
    #long arm cant stretch so pen and elbow move same amount along the arm.
    #R1.V = R1.U1 and R2.V = R2.U2. dot with R kills the swing part (perp to R).
    #RHS the known right side of those two lines.
    RHS = np.array([np.dot(R1, U1), np.dot(R2, U2)])
    #LHS coefficient of Vx,Vy. row1 = R1, row2 = R2.
    LHS = np.array([R1, R2])
    #2 eq 2 unknown. solve LHS*V = RHS.
    #det LHS -> 0 means R1 parallel R2, long arms in one line, type 2 singularity.
    return np.linalg.solve(LHS, RHS), np.linalg.det(LHS)

#Jacobian matrix. col1 = V for omega (1,0), col2 = V for omega (0,1).
#then V = J @ [omega1,omega2] for any omega.
def Jacobian(teta1,teta2):
    r1 = PenVelocity(teta1,teta2,1,0)
    r2 = PenVelocity(teta1,teta2,0,1)
    if r1 is None or r2 is None : return None
    return np.column_stack([r1[0], r2[0]]), r1[1]

#reverse. wanted pen velocity V -> omega1,omega2 rad/s.
#each line has only its own omega so no inverse needed.
#omega = (R.V) / (a * (R . elbow direction)). divisor -> 0 is type 1 singularity, arm straight.
def ServoSpeeds(teta1,teta2,V):
    B1,B2,P=FK(teta1,teta2)
    if(B1 is None) : return None
    R1,R2=P-B1,P-B2
    t1rad=np.radians(teta1)
    t2rad=np.radians(teta2)
    c1 = a * np.dot(R1, np.array([-np.sin(t1rad), np.cos(t1rad)]))
    c2 = a * np.dot(R2, np.array([-np.sin(t2rad), np.cos(t2rad)]))
    return np.dot(R1, V) / c1, np.dot(R2, V) / c2

#checks, numbers from the tutorial.
#expect [0. 107.28]
print("FK(120,60):", np.round(FK(teta1,teta2)[2],2))
P = np.array([30.0, 90.0])
t1, t2 = IK(P,A1,+1), IK(P,A2,-1)
#expect 108.28 29.42
print("IK(30,90):", round(t1,2), round(t2,2))
#expect [30. 90.]
print("loop check:", np.round(FK(t1,t2)[2],2))
#expect [-45.32 -35.15]
print("servo1 alone at (120,60):", np.round(PenVelocity(teta1,teta2,1,0)[0],2))
#expect [[-51.14 -28.32] [-29.71 52.93]] and 4897
J, detA = Jacobian(t1,t2)
print("J at (30,90):\n", np.round(J,2), "\ndet A:", round(detA,1))
#expect [-85.5 -48.0]
print("omega for V=(100,0) in deg/s:", np.round(np.degrees(ServoSpeeds(t1,t2,[100,0])),1))