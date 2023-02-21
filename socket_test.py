import socket
import numpy as np
import Cuplib as Cup
import time
HOST = '192.168.100.170'
PORT = 7000

cL = [8, 5, 4, 2, 1, 0]
Cup.ArmInit(cL)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(5)

print('server start at: %s:%s' % (HOST, PORT))
print('wait for connection...')

while True:
    conn, addr = s.accept()
    print('connected by ' + str(addr))

    while True:
        indata = conn.recv(1024)
        text = indata.decode()
        print("rec data:", text)
        if text[0:3]=="pos":
            pos = text.replace("pos", "")
            pos = pos[1:len(pos)-1].split(", ")
            pos = np.array(pos, dtype=int)
            Cup.tMove(cL, pos, 1500)
            time.sleep(0.5)

        outdata = "complete!"
        conn.send(outdata.encode())
s.close()
