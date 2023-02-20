from simulator import Arm
import Cuplib as Cup
import time
channelList = [8, 5, 4, 2, 1, 0]
Cup.Setpause(channelList)

arm = Arm(105, 90, 65, 105, opt=Arm.minimum_change, implementation="t")

LastC=[90, 90, 90, 90, 90, 90]

arm.goto(50, 0, 250)

print(arm.x, arm.y, arm.z)
for i in arm.position:
    print(int(i))

for i in range(len(channelList)):
    Cup.tMove(channelList[i], LastC[i], int(arm.position[i]), 0.05)
    LastC[i] = int(arm.position[i])

arm.goto(50, 0, 150)

print(arm.x, arm.y, arm.z)
for i in arm.position:
    print(int(i))

for i in range(len(channelList)):
    Cup.tMove(channelList[i], LastC[i], int(arm.position[i]), 0.05)
    LastC[i] = int(arm.position[i])

arm.goto(0, 50, 150)

print(arm.x, arm.y, arm.z)
for i in arm.position:
    print(int(i))

for i in range(len(channelList)):
    Cup.tMove(channelList[i], LastC[i], int(arm.position[i]), 0.05)
    LastC[i] = int(arm.position[i])


arm.goto(0, 50, 250)

print(arm.x, arm.y, arm.z)
for i in arm.position:
    print(int(i))

for i in range(len(channelList)):
    Cup.tMove(channelList[i], LastC[i], int(arm.position[i]), 0.05)
    LastC[i] = int(arm.position[i])


time.sleep(3)
for i in range(len(channelList)):
    Cup.tMove(channelList[i], int(arm.position[i]), 90, 0.05)


