from Solver import *
LIMIT = 300
SPEED = 100
dirs = [np.array([np.cos(t), np.sin(t)]) for t in np.radians(range(0,180,10))]

for y in range(125,25,-5):
    t1,t2 = IK(np.array([0.0,y]),A1,+1), IK(np.array([0.0,y]),A2,-1)
    J,D = Jacobian(t1,t2)
    detJ = np.linalg.det(J)
    wmax = max(np.max(np.abs(np.degrees(ServoSpeeds(t1,t2,SPEED*d)))) for d in dirs)
    cond = np.linalg.cond(J)
    step = np.linalg.norm(J @ np.radians([1,0]))
    print("------------------")
    print("For y point of :", y)
    print("D = det(LHS), long arm x long arm (0 = trap, type 2)  :", round(D))
    print("det(J) = c1*c2/D, short arm x long arm (0 = wall, type 1):", round(detJ))
    print("Worst servo speed for pen at", SPEED, "mm/s (deg/s)     :", round(wmax))
    print("Resolution: pen mm per 1 deg of servo 1               :", round(step,1))
    print("cond(J), 1 = ideal                                   :", round(cond,1))
    print("Safe ? :", "OK" if wmax<LIMIT and cond<5 else "NO")