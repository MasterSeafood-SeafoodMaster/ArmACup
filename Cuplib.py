import time
import threading as trd
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

def Setpause(channelList):
    for ch in channelList:
        kit.servo[ch].set_pulse_width_range(500, 2500)


def move(index, From, To, sleep):
    D=0
    print("Index", index, "From", From, "to", To)
    if (From>To): D=-1
    elif(To>From): D=1
    if(D!=0):
        for i in range(From, To, D):
            kit.servo[index].angle = i
            #print(i)
            time.sleep(sleep)
    time.sleep(0.5)


def tMove(index, From, To, sleep):
    t = trd.Thread(target=move, args=[index, From, To, sleep])
    t.start()
