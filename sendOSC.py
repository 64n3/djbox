import random
from pythonosc import udp_client
import time
import math
from scipy.signal import butter, lfilter
import numpy as np

if __name__ == '__main__':
    t_update = 0.5  # sleep time between distance measurements
    client_1 = udp_client.SimpleUDPClient("127.0.0.1", 1024)  # client for port 1024
    client_2 = udp_client.SimpleUDPClient("127.0.0.1", 1025)  # client for port 1025

    # define data lowpass filter
    f_cutoff = 1
    w_cutoff = f_cutoff * t_update
    b, a = butter(5, w_cutoff, btype='low', analog=False)

    # define scenario (1: random, 2: exponential decay)
    scen = 1
    x = []
    i = 0
    try:
        while True:
            if scen == 1:  # random numbers between 1000 and 2000
                x.append(np.random.uniform())  # generate random distance measurements
            elif scen == 2:  # exponential decay
                x.append(math.exp(-i/10)*1000)
                i+=1

            y = lfilter(b, a, x)
            dist = y[-1]

            client_1.send_message("/", dist)
            client_2.send_message("/", 1-dist)
            print(dist)
            time.sleep(t_update)
    except KeyboardInterrupt:
        print("Done\n")
