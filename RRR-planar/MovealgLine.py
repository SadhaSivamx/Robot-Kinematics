from Planar import *
from Superfunctions import *
import serial
import time


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

    ser = None

    try:
        print(f"Attempting to connect to {ARDUINO_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2)
        ser.flushInput()
        print(f"Successfully connected to {ARDUINO_PORT}.")

        print("Moving From (25,25) to (15,15)...")
        for pt in range(25,14,-1):
            ans=[]
            xpos, ypos, phi = 20,pt,0
            ypos_ik = ypos - C
            phi_rad = deg2rad(phi)
            solutions = IK(L1, L2, L3, xpos, ypos_ik, phi_rad)
            for i, (t1, t2, t3) in enumerate(solutions):
                absdeg1 = rad2deg(t1)
                absdeg2 = -rad2deg(t2)
                absdeg3 = 90 + rad2deg(t3)
                ans.append([absdeg1,absdeg2,absdeg3])
            results = Moveme(ser,ans[-1])

            if results:
                print(f"\nFunction completed. Final responses:")
                print(f"  M1 Response: {results[0]}")
                print(f"  M2 Response: {results[1]}")
                print(f"  M4 Response: {results[2]}")
            time.sleep(0.5)

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
