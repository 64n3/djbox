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

def calc_biquad(fc=1, fs=10, Q=1/np.sqrt(2)):
    w0 = 2 * np.pi * fc/fs
    a = np.sin(w0)/2*Q
    
    b0 = (1-np.cos(w0))/2
    b1 = 1 - np.cos(w0)
    b2 = b0
    
    a0 = 1+a
    a1 = -2 * np.cos(w0)
    a2 = 1-a
    
    vB = [b0/a0, b1/a0, b2/a0]
    vA = [0, a1/a0, a2/a0]
    
    return vB,vA

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
    fs_read = 100  # sampling frequency in Hz for sensors and smoothing
    t_read = 1/fs_read
    
    trig_dist = 500  # trigger distance in mm
    
    c_setup = udp_client.SimpleUDPClient(ip, 1024)  # setup data client
    c_trigger = udp_client.SimpleUDPClient(ip, 1025)  # sensor data client 1
    
    # patch setup
    t_update = 1.5  # interpolation time between trigger events in s
    f_min = 1000  # minimum frequency for lowpass filter in Hz
    max_attn = -60  # maximum attenuation in dB
    a_min = np.power(10, max_attn/20)  # minimum amplitude (linear)
    
    # send setup to Pd
    c_setup.send_message("/t_update",t_update)
    c_setup.send_message("/f_min",f_min)
    c_setup.send_message("/a_min",a_min)
    c_setup.send_message("/setup_done",1)
    
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
    vB, vA = calc_biquad(fc=1, fs=1/t_update);
    
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
