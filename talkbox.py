# import random
from pythonosc import udp_client
import RPi.GPIO as GPIO
import time
import math
from scipy.signal import butter, lfilter
import numpy as np
from numpy_ringbuffer import RingBuffer

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

def distmeas(GPIO_TRIGGER, GPIO_ECHO):
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
    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO_TRIGGER_1 = 23
    GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
    GPIO_ECHO_1 = 24
    GPIO.setup(GPIO_ECHO_1, GPIO.IN)
    
    # Network/Communication setup
    ip = "192.168.178.20"
    fs_read = 100  # sampling frequency in Hz for sensors and smoothing
    t_read = 1/fs_read
    
    trig_dist = 500  # trigger distance in mm
    
    c_setup = udp_client.SimpleUDPClient(ip, 1024)  # setup data client
    c_trigger = udp_client.SimpleUDPClient(ip, 1025)  # sensor data client 1
    
    # Patch setup
    t_update = 1.5  # interpolation time between trigger events in s
    f_min = 1000  # minimum frequency for lowpass filter in Hz
    max_attn = -60  # maximum attenuation in dB
    a_min = np.power(10, max_attn/20)  # minimum amplitude (linear)
    
    c_setup.send_message("/t_update",t_update)
    c_setup.send_message("/f_min",f_min)
    c_setup.send_message("/a_min",a_min)
    c_setup.send_message("/setup_done",1)
    
    # Limits
    trig_dist = 500  # trigger distance in mm
    
    # Ringbuffers to store measurements
    xR1 = RingBuffer(capacity=3)
    yR1 = RingBuffer(capacity=3)
    
    for i in range(3):  # initialize ringbuffer with zeros
        xR1.append(0)
        yR1.append(0)
    # Biquad coefficients
    vB, vA = calc_biquad(fc=1, fs=fs_read);
    
    # define scenario (1: random, 2: exponential decay)
    scen = 0
    i = 0
    try:
        trigger = 0
        while True:
            xR1.appendleft(distmeas(GPIO_TRIGGER_1, GPIO_ECHO_1))
            
            y1 = vB[0] * xR1[0] + vB[1] * xR1[1] + vB[2] * xR1[2] - vA[1] * yR1[0] - vA[2] * yR1[1]
            yR1.appendleft(y1)
            
            # detect trigger 
            if y1 > trig_dist and trigger != 0:
                trigger = 0
                c_trigger.send_message("/", trigger)
            elif y1 <= trig_dist and trigger != 1:
                trigger = 1
                c_trigger.send_message("/", trigger)
            else:
                time.sleep(t_read)
                
            print("{}mm --> t={}".format(y1,trigger))
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Done\n")
