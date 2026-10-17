import numpy as np
import matplotlib.pyplot as plt
import serial as Serial
import time,warnings

def Cross(v1,v2):
    return v1[0]*v2[1] - v1[1]*v2[0]

# def sendcmd(sL, sR):
#     msg = f"M1:{sL:.2f},M2:{sR:.2f}\n"
#     arduino.write(msg.encode())
#     return arduino.readline().decode().strip()

# PORT = "COM6"
# arduino = Serial.Serial(PORT, 9600, timeout=1)
# time.sleep(2)

origin = np.array([0, 0])
dlink = 25.8
a,b = 60, 70
teta1, teta2 = 96, 3

A1 = origin + np.array([-dlink / 2, 0])
A2 = origin + np.array([dlink / 2, 0])

print(A1,A2)

t1rad = np.radians(teta1)
t2rad = np.radians(teta2)

B1 = A1 + np.array([a * np.cos(t1rad), a * np.sin(t1rad)])
B2 = A2 + np.array([a * np.cos(t2rad), a * np.sin(t2rad)])
L = np.linalg.norm(B2-B1)

if (L>2*b or Cross(B1,B2)>0):
    print("Not Possible to get to this configuration.")
else:
    Mp = (B1 + B2) / 2
    h = np.sqrt(b ** 2 - (L / 2) ** 2)

    VB1B2 = B2 - B1
    Pvector = np.array([-VB1B2[1], VB1B2[0]])
    Pcap = Pvector / np.linalg.norm(Pvector)

    P = Mp + Pcap * h
    print("Point P coordinates:", P)

    #sendcmd(teta2, teta1)

    plt.figure(figsize=(8, 6))
    plt.plot([A1[0], A2[0]], [A1[1], A2[1]], 'ko-', label='Ground')
    plt.plot([A1[0], B1[0]], [A1[1], B1[1]], 'ro-', label='a1')
    plt.plot([A2[0], B2[0]], [A2[1], B2[1]], 'bo-', label='a2')
    plt.plot([B1[0], P[0]], [B1[1], P[1]], 'm--', label='b1')
    plt.plot([B2[0], P[0]], [B2[1], P[1]], 'c--', label='b2')
    plt.scatter(*P, color='orange', zorder=5, s=100, label='End Point P')
    plt.axis('equal')
    plt.grid(True)
    plt.xlim(-150,150)
    plt.ylim(-10,150)
    plt.legend()
    plt.title("Parallelx")
    plt.show()
