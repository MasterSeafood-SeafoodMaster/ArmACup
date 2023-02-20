import time
import Cuplib as Cup
import random
#channelList = [8, 5, 4, 2, 1, 0]
channelList = [0, 1, 2, 4, 5, 8]
lastCup = [90, 90, 90, 90, 90, 90]
Cup.Setpause(channelList)

for t in range(60):
    for i in range(len(channelList)):
        r = random.randint(60, 120)
        Cup.tMove(channelList[i], lastCup[i], r, 0.01)
        lastCup[i] = r
    time.sleep(1)
for i in range(len(channelList)):
    Cup.tMove(channelList[i], lastCup[i], 90, 0.05)


