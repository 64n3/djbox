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
    print("DBG:setup sensors")
    GPIO.setmode(GPIO.BCM)
    GPIO_TRIGGER_1 = 22
    GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
    GPIO_TRIGGER_2 = 27
    GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
    GPIO_TRIGGER_3 = 17
    GPIO.setup(GPIO_TRIGGER_3, GPIO.OUT)
    GPIO_ECHO_1 = 21
    GPIO.setup(GPIO_ECHO_1, GPIO.IN)
    GPIO_ECHO_2 = 20
    GPIO.setup(GPIO_ECHO_2, GPIO.IN)
    GPIO_ECHO_3 = 16
    GPIO.setup(GPIO_ECHO_3, GPIO.IN)
    
    # Network/Communication setup
    print("DBG:setup network")
    ip = "192.168.178.36"
    fs_read = 100  # sampling frequency in Hz for sensors and smoothing
    t_read = 1/fs_read
    
    trig_dist = 500  # trigger distance in mm
    
    c_setup = udp_client.SimpleUDPClient(ip, 1024)  # setup data client
    c_trigger = udp_client.SimpleUDPClient(ip, 1025)  # sensor data client 1
    
    # Patch setup
    print("DBG:setup patch")
    t_update = 1.5  # interpolation time between trigger events in s
    send_comp = t_update/t_read  # compensation between reading and updating
    f_min = 1000  # minimum frequency for lowpass filter in Hz
    max_attn = -60  # maximum attenuation in dB
    a_min = np.power(10, max_attn/20)  # minimum amplitude (linear)
    f_low = 300  # center frequency of low shelf filter
    g_low = -3  # gain of low shelf filter
    g_mid = -10  # gain of mit hml filter
    f_high = 2000  # center frequency of high shelf filter
    g_high = -8  # gain of high shelf filter
    
    c_setup.send_message("/t_update",t_update)
    c_setup.send_message("/f_min",f_min)
    c_setup.send_message("/a_min",a_min)
    c_setup.send_message("/f_low",f_low)
    c_setup.send_message("/g_low",g_low)
    c_setup.send_message("/g_mid",g_mid)
    c_setup.send_message("/f_high",f_high)
    c_setup.send_message("/g_high",g_high)
    c_setup.send_message("/setup_done",1)
    
    # Limits
    trig_dist = 500  # trigger distance in mm
    
    # Ringbuffers to store measurements
    print("DBG:setup ringbuffers")
    n_buff = 3
    xR1 = RingBuffer(capacity=n_buff)
    xR2 = RingBuffer(capacity=n_buff)
    xR3 = RingBuffer(capacity=n_buff)
    yR1 = RingBuffer(capacity=n_buff)
    yR2 = RingBuffer(capacity=n_buff)
    yR3 = RingBuffer(capacity=n_buff)
    
    for i in range(3):  # initialize ringbuffer with zeros
        xR1.append(0)
        xR2.append(0)
        xR3.append(0)
        yR1.append(0)
        yR2.append(0)
        yR3.append(0)
        
    # Biquad coefficients
    vB, vA = calc_biquad(fc=1, fs=fs_read);
    
    # define scenario (1: random, 2: exponential decay)
    scen = 0
    i = 0
    try:
        trigger = 0
        i_read = 0  # counter for read cycles, before send event
        T = [0,0,0]
        while True:
            xR1.appendleft(distmeas(GPIO_TRIGGER_1, GPIO_ECHO_1))
            xR2.appendleft(distmeas(GPIO_TRIGGER_2, GPIO_ECHO_2))
            xR3.appendleft(distmeas(GPIO_TRIGGER_3, GPIO_ECHO_3))
            
            y1 = vB[0] * xR1[0] + vB[1] * xR1[1] + vB[2] * xR1[2] - vA[1] * yR1[0] - vA[2] * yR1[1]
            y2 = vB[0] * xR2[0] + vB[1] * xR2[1] + vB[2] * xR2[2] - vA[1] * yR2[0] - vA[2] * yR2[1]
            y3 = vB[0] * xR3[0] + vB[1] * xR3[1] + vB[2] * xR3[2] - vA[1] * yR3[0] - vA[2] * yR3[1]
            
            yR1.appendleft(y1)
            yR2.appendleft(y2)
            yR3.appendleft(y3)
            
            # evaluate distances and trigger value
            Y = [y1, y2, y3]
            for ii in range(len(Y)):
                T[ii] = int(Y[ii] < trig_dist)
            trigger = sum(T)-1
            if trigger < 0: trigger = 0
            
            i_read += 1
            if i_read == send_comp:
                c_trigger.send_message("/", trigger)
                i_read = 0
            
            
#             print("{} | {} | {}".format(y1, y2, y3))
#             print("{} | {} | {} --> {}".format(T[0],T[1],T[2], trigger))
            
#             # detect trigger 
#             if y1 > trig_dist and trigger != 0:
#                 trigger = 0
#                 c_trigger.send_message("/", trigger)
#             elif y1 <= trig_dist and trigger != 1:
#                 trigger = 1
#                 c_trigger.send_message("/", trigger)
#             else:
#                 time.sleep(t_read)
#                 
#             print("{}mm --> t={}".format(y1,trigger))


    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Done\n")
