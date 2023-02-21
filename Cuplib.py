import time
import threading as trd
from adafruit_servokit import ServoKit
import random
import numpy as np

kit = ServoKit(channels=16)

def ArmInit(channelList):    
    for ch in channelList:
        kit.servo[ch].set_pulse_width_range(500, 2500)
    setZero(channelList)

def setZero(channelList):
    print("set zero")
    for ch in channelList:
        t = trd.Thread(target=move, args=[ch, kit.servo[ch].angle, 90, 2000])  
        t.start()
    time.sleep(2.5)

def move(index, From, To, step):  
    mAry = np.linspace(From, To, step, dtype=int)
    lastAng = 180
    for i in mAry:
        if abs(lastAng-i)>=1:
            kit.servo[index].angle = i
            lastAng = i
        time.sleep(0.001)

def tMove(cList, tList, step):
    for cidx in range(len(cList)):
        La = int(kit.servo[cList[cidx]].angle)
        t = trd.Thread(target=move, args=[cList[cidx], La, tList[cidx], step])
        t.start()
    time.sleep(step*0.001)

def getRandList(Min, Max):
    tList = []
    for i in range(6):
        tList.append(random.randint(Min, Max))
    return tList
    
