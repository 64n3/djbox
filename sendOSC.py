import random
from pythonosc import udp_client
import time
import math
from scipy.signal import butter, lfilter
import numpy as np

if __name__ == '__main__':
    t_update = 0.5  # sleep time between distance measurements
    client = udp_client.SimpleUDPClient("127.0.0.1", 1111)  # send to port 1111 on localhost

    # define data lowpass filter
    f_cutoff = 1
    w_cutoff = f_cutoff * t_update
    b, a = butter(5, w_cutoff, btype='low', analog=False)

    # define scenario (1: random, 2: exponential decay)
    scen = 2
    x = []
    i = 0
    try:
        while True:
            if scen == 1:  # random numbers between 1000 and 2000
                x.append(1000+random.randint(0,1000))  # generate random distance measurements
            elif scen == 2:  # exponential decay
                x.append(math.exp(-i/10)*1000)
                i+=1

            y = lfilter(b, a, x)
            dist = y[-1]

            client.send_message("/", dist)
            print(dist)
            time.sleep(t_update)
    except KeyboardInterrupt:
        print("Done\n")
