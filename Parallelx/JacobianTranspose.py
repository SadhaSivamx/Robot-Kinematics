import matplotlib.pyplot as plt
import numpy as np
from Solver import *

teta = np.array([120.0, 60.0])
TargetPoint = np.array([25.0, 80.0])
xpts = []
ypts = []

threshold = 1e-5
alpha = 2e-4

for k in range(200):
    B1, B2, CurrentPoint = FK(*teta)
    e = TargetPoint - CurrentPoint

    E = 0.5 * np.sum(e**2)

    print("----------------------------------")
    print("For the {} - th iteration.".format(k))
    print(k, np.round(teta,0), np.round(CurrentPoint, 1), round(E, 2))
    if E < threshold: break

    Jm, _ = Jacobian(*teta)
    update = Jm.T @ e
    teta += np.degrees(alpha * update)

    xpts.append(k);ypts.append(E)

print("Servo Needs to be at",np.round(teta,0),"for reaching",TargetPoint)
plt.plot(xpts,ypts,c='b')
plt.scatter(xpts,ypts,c='r')
plt.title("Iteration vs Error Curve")
plt.ylabel("Error")
plt.xlabel("Iteration")
plt.show()