import random
from pythonosc import udp_client
import time

if __name__ == '__main__':
    client = udp_client.SimpleUDPClient("127.0.0.1", 1111)
    try:
        while True:
            n = 1000+random.randint(0,1000)
            client.send_message("/", n)
            print(n)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Done\n")
