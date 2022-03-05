import time
import socket
import math
import numpy as np


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('127.0.0.1', 8888))
delay = 10  # ms
while True:
    print("Waiting...")
    s.listen()
    conn, addr = s.accept()
    print("Connected")
    cn = 0
    t = 0
    while True:
        try:
            a = int((math.sin(cn) + 5000) * 10000)
            b = int((math.sin(cn + 2 * math.pi / 3) + 5000) * 10000)
            c = int((math.sin(cn + 4 * math.pi / 3) + 5000) * 10000)
            # cn += 0.1
            cn += np.random.rand() / 10
            t += delay
            data = ((t + 5000) * 10000).to_bytes(4, 'big') + \
                   a.to_bytes(4, 'big') + \
                   b.to_bytes(4, 'big') + \
                   c.to_bytes(4, 'big') + \
                   a.to_bytes(4, 'big') + \
                   b.to_bytes(4, 'big') + \
                   c.to_bytes(4, 'big') + \
                   a.to_bytes(4, 'big') + \
                   b.to_bytes(4, 'big') + \
                   c.to_bytes(4, 'big') + \
                   (6).to_bytes(1, 'big')
            conn.send(data)
            # print(int.from_bytes(data[0:4], 'big') / 10000 - 5000)
            # print(data)
            time.sleep(delay / 1000)
        except Exception as e:
            # print(e)
            print('Disconnected')
            break
