import time
import Cuplib as Cup

#channelList = [8, 5, 4, 2, 1, 0]
channelList = [0, 1, 2, 4, 5, 8]

Cup.Setpause(channelList)
for i in channelList:
    for j in range(1):
        Cup.move(i, 90, 120, 0.01)
        Cup.move(i, 120, 60, 0.01)
        Cup.move(i, 60, 120, 0.01)
        Cup.move(i, 120, 90, 0.01)

