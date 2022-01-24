# import random
from pythonosc import udp_client
import RPi.GPIO as GPIO
import time
import math
from scipy.signal import butter, lfilter
import numpy as np
from numpy_ringbuffer import RingBuffer

# set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 23
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO_ECHO = 24
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distmeas():
    # trigger measurement
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    
    StartTime = time.time()
    StopTime = time.time()
    
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        
    TimeElapsed = StopTime-StartTime
    distance = TimeElapsed * 34300 / 2 * 10  # distance in mm
    return distance
    

if __name__ == '__main__':
    ip = "192.168.178.36"
    t_update = 0.1  # sleep time between distance measurements
    client_0 = udp_client.SimpleUDPClient(ip, 1024)  # setup data client
    client_1 = udp_client.SimpleUDPClient(ip, 1025)  # sensor data client 1
    client_2 = udp_client.SimpleUDPClient(ip, 1026)  # sensor data client 2

    # send setup to Pd
    client_0.send_message("/t_update",t_update*1000)
    client_0.send_message("/setup_done",1)
    
    # limits
    max_dist = 400  # mm
    min_dist = 20  # mm
    dist_range = max_dist-min_dist  # mm
    
    # ringbuffer to store measurements
    xR = RingBuffer(capacity=3)
    yR = RingBuffer(capacity=3)
    
    for i in range(3):  # initialize ringbuffer with zeros
        xR.append(0)
        yR.append(0)
    # Biquad coefficients
    vA = [0, -1.25051643, 0.54572332]  # [a0, a1, a2]
    vB = [0.07380173, 0.14760344, 0.07380172]   # [b0, b1, b2]
    
    # define scenario (1: random, 2: exponential decay)
    scen = 0
    i = 0
    try:
        while True:
            if scen == 0:  # use sensors for measurement
                xR.appendleft(distmeas())
            elif scen == 1:  # random numbers between 1000 and 2000
                xR.appendleft(np.random.uniform())  # generate random distance measurements
            elif scen == 2:  # exponential decay
                xR.appendleft(math.exp(-i/10)*1000)
                i+=1
            
            y = vB[0] * xR[0] + vB[1] * xR[1] + vB[2] * xR[2] - vA[1] * yR[0] - vA[2] * yR[1]
            yR.appendleft(y)
            
            # normalize data
            if y > max_dist: y = max_dist
            if y < min_dist: y = min_dist
            y = y / dist_range
            
            # send data via OSC
            client_1.send_message("/", y)
            client_2.send_message("/", 1-y)
            time.sleep(t_update)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Done\n")
