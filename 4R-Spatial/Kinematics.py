import math
import serial,time
import numpy as np


L1 = 15.5
L2 = 12
L3 = 12
C = 8.5

def getcircle(x,cx,cy,r):
    return int(cy+math.sqrt(r**2-(x-cx)**2))
def getcircleinv(x,cx,cy,r):
    return int(cy-math.sqrt(r**2-(x-cx)**2))
def deg2rad(d):
    return d * np.pi / 180.0

def rad2deg(r):
    return r * 180.0 / np.pi
def Moveme(ser, sol):
    try:
        t0 = sol[0]
        t1 = sol[1]
        t2 = sol[2]
        t3 = sol[3]

        commands = [
            f"M1:{t1}\n".encode('utf-8'),
            f"M2:{t2}\n".encode('utf-8'),
            f"M4:{t3}\n".encode('utf-8'),
            f"M0:{t0}\n".encode('utf-8'),
        ]

        responses = []
        print("--- Sending Motor Commands ---")

        for cmd in commands:
            ser.write(cmd)
            print(f"Sent: {cmd.decode('utf-8').strip()}")

            response = ser.readline().decode('utf-8').strip()

            print(f"Received: {response}")
            responses.append(response)

        print("--- All Commands Sent ---")
        return responses

    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
        return None
    except IndexError:
        print(f"Error: 'sol' list must contain 3 elements. Received: {sol}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred in Moveme: {e}")
        return None

def FK(l1, l2, l3, c, angdeg):

    base, th1, th2, th3 = np.radians(angdeg)

    th12 = th1 + th2
    th123 = th1 + th2 + th3

    r = l1 * np.cos(th1) + l2 * np.cos(th12) + l3 * np.cos(th123)
    z = l1 * np.sin(th1) + l2 * np.sin(th12) + l3 * np.sin(th123)

    X = r * np.cos(base)
    Y = r * np.sin(base)
    Z = z + c

    phideg = np.degrees(th123)
    return [round(X, 2), round(Y, 2), round(Z, 2), round(phideg, 2)]

def IK(l1, l2, l3, c_offset, x_target, y_target, z_target, Phi_angle_deg):

    base_angle_radians = np.arctan2(y_target, x_target+0.001)

    r_target = np.sqrt(x_target ** 2 + y_target ** 2)

    z_local_target = z_target - c_offset

    phi_angle_rad = np.radians(Phi_angle_deg)

    r_wrist = r_target - l3 * np.cos(phi_angle_rad)
    z_wrist = z_local_target - l3 * np.sin(phi_angle_rad)

    r_wrist_sq = r_wrist ** 2 + z_wrist ** 2
    r_wrist_dist = np.sqrt(r_wrist_sq)

    if r_wrist_dist > (l1 + l2) or r_wrist_dist < abs(l1 - l2):
        print(f"IK Error: Target wrist ({r_wrist:.2f}, {z_wrist:.2f}) is unreachable by L1+L2.")
        return None

    c2 = (r_wrist_sq - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    c2 = np.clip(c2, -1.0, 1.0)

    theta2_sol1_rad = np.arccos(c2)
    theta2_sol2_rad = -theta2_sol1_rad

    solutions_rad = []

    k1_s1 = l1 + l2 * np.cos(theta2_sol1_rad)
    k2_s1 = l2 * np.sin(theta2_sol1_rad)
    theta1_sol1_rad = np.arctan2(z_wrist, r_wrist+0.001) - np.arctan2(k2_s1, k1_s1+0.001)
    theta3_sol1_rad = phi_angle_rad - theta1_sol1_rad - theta2_sol1_rad

    sol1 = (base_angle_radians, theta1_sol1_rad, theta2_sol1_rad, theta3_sol1_rad)
    solutions_rad.append(sol1)

    if not np.isclose(theta2_sol1_rad, theta2_sol2_rad):
        k1_s2 = l1 + l2 * np.cos(theta2_sol2_rad)
        k2_s2 = l2 * np.sin(theta2_sol2_rad)
        theta1_sol2_rad = np.arctan2(z_wrist, r_wrist+0.001) - np.arctan2(k2_s2, k1_s2+0.001)
        theta3_sol2_rad = phi_angle_rad - theta1_sol2_rad - theta2_sol2_rad

        sol2 = (base_angle_radians, theta1_sol2_rad, theta2_sol2_rad, theta3_sol2_rad)
        solutions_rad.append(sol2)

    return solutions_rad


if __name__ == "__main__":
    ARDUINO_PORT = 'COM13'
    BAUD_RATE = 115200
    TIMEOUT = 2

    ser = None

    try:
        print(f"Attempting to connect to {ARDUINO_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2)
        print(f"Successfully connected to {ARDUINO_PORT}.")

        for pt in range(-10,11,1):
            ans=[]
            xpos,ypos,zpos,phi = pt, getcircle(pt, 0, 18, 10), 22, 0
            phi_rad = deg2rad(phi)
            solutions = IK(L1, L2, L3, C, xpos, ypos,zpos, phi)
            for i, (b, t1, t2, t3) in enumerate(solutions):
                absdeg1 = rad2deg(t1)
                absdeg2 = -rad2deg(t2)
                absdeg3 = 90 + rad2deg(t3)
                absdeg4 = rad2deg(b)
                ans.append([absdeg4,absdeg1,absdeg2,absdeg3])
            results = Moveme(ser,ans[-1])

            if results:
                print(f"\nFunction completed. Final responses:")
                print(f"  M1 Response: {results[0]}")
                print(f"  M2 Response: {results[1]}")
                print(f"  M4 Response: {results[2]}")
                print(f"  M0 Response: {results[3]}")
            time.sleep(0.25)

        for pt in range(10, -11, -1):
            ans = []
            xpos, ypos, zpos, phi = pt, getcircleinv(pt, 0, 18, 10), 22, 0
            phi_rad = deg2rad(phi)
            solutions = IK(L1, L2, L3, C, xpos, ypos, zpos, phi)
            for i, (b, t1, t2, t3) in enumerate(solutions):
                absdeg1 = rad2deg(t1)
                absdeg2 = -rad2deg(t2)
                absdeg3 = 90 + rad2deg(t3)
                absdeg4 = rad2deg(b)
                ans.append([absdeg4, absdeg1, absdeg2, absdeg3])
            results = Moveme(ser, ans[-1])

            if results:
                print(f"\nFunction completed. Final responses:")
                print(f"  M1 Response: {results[0]}")
                print(f"  M2 Response: {results[1]}")
                print(f"  M4 Response: {results[2]}")
                print(f"  M0 Response: {results[3]}")
            time.sleep(0.25)

    except serial.SerialException as e:
        print(f"Error: Could not open or use port {ARDUINO_PORT}.")
        print(f"Details: {e}")
        print("Please check that the Arduino is connected and the port is correct.")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    finally:
        if ser and ser.is_open:
            ser.close()
            print(f"\nSerial port {ARDUINO_PORT} closed.")
