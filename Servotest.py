import time
import Cuplib as Cup

cL = [8, 5, 4, 2, 1, 0]

Cup.ArmInit(cL)
t=0
while True:
    print("movement:", t)
    t += 1
    tList = Cup.getRandList(45, 135)
    Cup.tMove(cL, tList, 1500)
    time.sleep(0.5)
    


