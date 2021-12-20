#Libraries
import RPi.GPIO as GPIO
import time
from pythonosc import udp_client
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    client = udp_client.SimpleUDPClient("127.0.0.1", 1111)
    bExportData = True
    tFile = "recorded_data.txt"
    try:  # Run and write to file
        i = 0
        if bExportData:
           file = open(tFile, "w")
        #with open("recorded_data.txt", "w") as file:
        while True:
           dist = distance()
           print ("m{}: {:.1f} cm".format(i, dist))
           client.send_message("/", dist * 10)
           time.sleep(0.01)
           if bExportData:
             file.write("%f\n" % dist)
           i += 1
    except KeyboardInterrupt:  # Stop by pressinc CTRL+C
        print("Measurement stopped by User")
        print("Did %d runs" % i)
        if bExportData:
          print("Wrote {} lines to {}".format(i, tFile))
        GPIO.cleanup()
