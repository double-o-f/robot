#import RPi.GPIO as GPIO
#import smbus
import numpy as np
#import matplotlib as plt
import time
import math
import serial

# Define map size and resolution
MAP_SIZE = 1024  # Size of the map (1024x1024)
MAP_RESOLUTION = 7.62  # Each cell represents 3 in
occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE))  # Initialize occupancy grid

ser = None
# ARD_COM = ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2")
ARD_COM = ("COM6", )
tryCom = 0


def connectArd():
    global ser
    global tryCom
    tryInit = True
    while tryInit:
        try:
            ser = serial.Serial(ARD_COM[tryCom], 115200, timeout=.1)

        except KeyboardInterrupt:
            exit(130)

        except:
            print("could no connect to " + ARD_COM[tryCom])
            if tryCom >= len(ARD_COM) - 1:
                tryCom = 0
            else:
                tryCom += 1
            time.sleep(0.125)
        else:
            tryInit = False


def readStream():
    data = ser.readline().decode()#.strip()  # Read and decode
    if data != '':
        AG = None #ang
        DF = None #dist front
        DP = None #dist pan
        TP = None #amb temp
        TO = None #obj temp
        YA = None #yaw

        #print(data, end='')

        AG0 = data.find("AG:")
        AG1 = data.find(":AG")
        if AG0 != -1 and AG1 != -1:
            AG = int(data[AG0 + 3:AG1])
            #print(AG)

        DF0 = data.find("DF:")
        DF1 = data.find(":DF")
        if DF0 != -1 and DF1 != -1:
            DF = float(data[DF0 + 3:DF1])
            if DF == 0:
                DF = None
            #print(DF)

        DP0 = data.find("DP:")
        DP1 = data.find(":DP")
        if DP0 != -1 and DP1 != -1:
            DP = float(data[DP0 + 3:DP1])
            if DP == 0:
                DP = None
            #print(DP)

        TP0 = data.find("TP:")
        TP1 = data.find(":TP")
        if TP0 != -1 and TP1 != -1:
            TP = float(data[TP0 + 3:TP1])
            if math.isnan(TP):
                TP = None
            #print(TP)

        YA0 = data.find("YA:")
        YA1 = data.find(":YA")
        if YA0 != -1 and YA1 != -1:
            YA = int(data[YA0 + 3:YA1])
            #print(YA) 

        return (True, AG, DF, DP, TP, YA)
    return (False, None, None, None, None, None)
   #return (was bus read, survo angle, front ultrasonic, survo ultrasonic, temp, yaw)
   #None means read error


# def update_map(distance, angle):
#     global robot_x, robot_y

#     # Calculate the position in the grid
#     if distance < 100:  # Use a threshold to avoid noise TEST THIS
#         grid_x = int(robot_x + (distance / MAP_RESOLUTION) * math.cos(math.radians(angle)))
#         grid_y = int(robot_y + (distance / MAP_RESOLUTION) * math.sin(math.radians(angle)))

#         # Ensure within bounds of the map
#         if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
#             occupancy_grid[grid_y, grid_x] = 1  # Mark as occupied

        
#def visualize_map():
#    plt.imshow(occupancy_grid, cmap="gray_r", origin="lower")
#    plt.title("Occupancy Grid Map")
#    plt.pause(0.01)


def main():
    #plt.ion() 
    
    while(True):
        data = readStream()
        if data[0] == True:
            for i in data:        
                print(str(i) + ', ', end='')
            print()

connectArd()
#main()
while True:
    try:
        main()

    except KeyboardInterrupt:
        exit(130)

    except:
        print("attempting reconnect to " + ARD_COM)
        ser.close()
        connectArd()
