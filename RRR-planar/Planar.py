import numpy as np
import matplotlib.pyplot as plt
from Superfunctions import *
import serial
import time

def FK(l1, l2, l3, theta1, theta2, theta3):
    theta12 = theta1 + theta2
    theta123 = theta1 + theta2 + theta3
    x = l1 * np.cos(theta1) + l2 * np.cos(theta12) + l3 * np.cos(theta123)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta12) + l3 * np.sin(theta123)
    phi = theta123
    return x, y, phi
def IK(l1, l2, l3, x_target, y_target, phi_target):
    xw = x_target - l3 * np.cos(phi_target)
    yw = y_target - l3 * np.sin(phi_target)

    r_sq = xw ** 2 + yw ** 2
    r = np.sqrt(r_sq)

    if r > (l1 + l2) or r < abs(l1 - l2):
        return None

    solutions = []

    cos_theta2 = (r_sq - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2_sol1 = np.arccos(cos_theta2)
    theta2_sol2 = -theta2_sol1

    k1_sol1 = l1 + l2 * np.cos(theta2_sol1)
    k2_sol1 = l2 * np.sin(theta2_sol1)
    theta1_sol1 = np.arctan2(yw, xw) - np.arctan2(k2_sol1, k1_sol1)
    theta3_sol1 = phi_target - theta1_sol1 - theta2_sol1

    solutions.append((theta1_sol1, theta2_sol1, theta3_sol1))

    if not np.isclose(theta2_sol1, theta2_sol2):
        k1_sol2 = l1 + l2 * np.cos(theta2_sol2)
        k2_sol2 = l2 * np.sin(theta2_sol2)
        theta1_sol2 = np.arctan2(yw, xw) - np.arctan2(k2_sol2, k1_sol2)
        theta3_sol2 = phi_target - theta1_sol2 - theta2_sol2
        solutions.append((theta1_sol2, theta2_sol2, theta3_sol2))
    return solutions

def Moveme(ser, sol):
    try:
        t1 = sol[0]
        t2 = sol[1]
        t3 = sol[2]

        commands = [
            f"M1:{t1}\n".encode('utf-8'),
            f"M2:{t2}\n".encode('utf-8'),
            f"M4:{t3}\n".encode('utf-8')
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



if __name__ == "__main__":
    ARDUINO_PORT = 'COM13'
    BAUD_RATE = 115200
    TIMEOUT = 2

    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2)
    ser.flushInput()
    print(f"Successfully connected to {ARDUINO_PORT}.")

    user_input = input("Enter Req Pos (x y phi): ")
    xpos, ypos, phi = list(map(float, user_input.split()))

    ypos_ik = ypos - C
    phi_rad = deg2rad(phi)

    solutions = IK(L1, L2, L3, xpos, ypos_ik, phi_rad)

    print(f"Target x:{xpos} y:{ypos} phi:{phi}")
    if solutions:
        print(f"Found {len(solutions)} solution(s):")
        ans=[]
        for i, (t1, t2, t3) in enumerate(solutions):
            absdeg1=rad2deg(t1)
            absdeg2=-rad2deg(t2)
            absdeg3=90+rad2deg(t3)
            print(f"  Solution {i + 1} (Motor Commands, deg): "
                  f"t1={absdeg1:.2f}, "
                  f"t2={absdeg2:.2f}, "
                  f"t3={absdeg3:.2f}")
            ans.append([absdeg1,absdeg2,absdeg3])

        plot_solutions(L1, L2, L3, C, solutions, xpos, ypos, phi_rad)

        if input("Want to Perform Motion? y/n : ")=="y":
            solution_to_move = ans[-1]
            results = Moveme(ser, solution_to_move)
            if results:
                print(f"\nFunction completed. Final responses:")
                print(f"  M1 Response: {results[0]}")
                print(f"  M2 Response: {results[1]}")
                print(f"  M4 Response: {results[2]}")
        else:
            print("Exiting...")
    else:
        print("No solutions Found...")