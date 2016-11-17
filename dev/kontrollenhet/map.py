from UART import UART
from modules import *
from sensorenhet_functions import *
import json
import math, time
import logging
import matplotlib.pyplot as plt
import numpy as np
import cv2
from math import atan2, degrees, pi

logging.getLogger(__name__).setLevel(logging.INFO)


grid = []


def read_debug_data():
    """
    Reads old measurements that have been saved to a json file
    """
    with open('demo_data/triple_sided_wall_with_imperfections.json') as data_file:    
        data = json.load(data_file)
    return data

def measure_lidar():
    """
    Turns servo 180 degrees while scanning and converts the scanned points to cordinates.
    Returns measurements in [[x1,y2],[x2,y2], ...]
    """
    uart = UART("ttyUSB1")

    sensorunit = UART("ttyUSB0")
    driveInstruction = Servo(0)

    degree_plot = []
    distance_plot = []

    uart.send_function(driveInstruction)
    time.sleep(2)
    degree = 0

    measurements = []

    for degree in range(0,180):
        sensorunit.send_function(ReadLidar())
        
        
        if sensorunit.decode_metapacket(sensorunit.receive_packet())[2]!=0:
            raise Exception("Incorrect acknowledge packet")
        if sensorunit.decode_metapacket(sensorunit.receive_packet())[2]!=8:
            raise Exception("Not a lidar value")

        highest = sensorunit.receive_packet()
        lowest = sensorunit.receive_packet()
        dist = ord(lowest)+ord(highest)*(2**8)

        logging.info("distance", dist)
        logging.info("move to", degree)

        # Read controller info
        #uart.send_function(ControllerInformation())
        #for i in range(6):
        #    lsb = uart.receive_packet()
        #print (ord(lsb))

        x = math.sin(math.radians(degree))*dist
        y = math.cos(math.radians(degree))*dist

        measurements.append([degree,dist])

        uart.send_function(Servo(int(degree)))

    return measurements

def plot_measurement(measurements, plotter = "matplot"):
    """
    Plots the measurements using a plotter of choise. cv2 will also try to estimate walls
    """
    degree_plot = []
    distance_plot = []
    

    if plotter == "matplot":
        for degree,dist in measurements:
            #print (x,y)
            x = math.sin(math.radians(degree-90))*dist
            y = math.cos(math.radians(degree-90))*dist
            degree_plot.append(degree)
            distance_plot.append(dist)

        plt.scatter(degree_plot,distance_plot)
        plt.show() 

    elif plotter == "cv2":
        # Create empty (oversized for most cases) image.
        img = np.zeros((512*4,512*4,3), np.uint8)


        degree_plot = []
        distance_plot = []


        # Fill in image with cirles at the points specified in input.
        for x,y in measurements:
            print (x,y)
            degree_plot.append(x)
            distance_plot.append(y)
            # Move circles to deal with minus values. Diameter 20, color white
            cv2.circle(img,(int(x+512),int(y+1024)), 20, (255,255,255), -1)



        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # See http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#houghlinesp. 
        lines = cv2.HoughLinesP(gray,1,np.pi/180,threshold=7,minLineLength=300,maxLineGap=20)
        print (lines)

        # Draw lines found
        for x0,y0,x1,y1 in lines[0]:
            print (x0,x1,y0,y1)
            dx = x1 - x0
            dy = y1 - y0
            rads = atan2(-dy,dx)
            rads %= 2*pi
            degs = degrees(rads)
            print (degs)
            # Filter lines that are not 0,90,180,270 degrees angle (+/-30)
            for targetAngle in range (0,361,90):
                print (targetAngle)
                if degs<targetAngle+30 and degs>targetAngle-30:
                    cv2.line(img,(x0,y0),(x1,y1),(0,0,255),2)

        # Show and close on buttonpress
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()




def loop_through_grid(measurements):
    """
    Early test for a grid based wall-finding system.
    For now just counts the number of measurement points located in specific intervals. 
    (Plot measurements to calculate appropsiate starting points)
    """
    for target in range(-400, 1200,400):
        found = 0
        for x, y in measurements:
            if y<target+100 and y> target-100:
                found +=1
        print ("found", found , "at", target)




# Uncomment to make new measurements
#plotdata = measure_lidar()
#print (plotdata)
#plot_measurement(plotdata)
#with open('perfect_square_center_raw_data.json', 'w') as outfile:
#    json.dump(plotdata, outfile)


loop_through_grid(read_debug_data())
plot_measurement(read_debug_data(), plotter="matplot")