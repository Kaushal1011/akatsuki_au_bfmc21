import socket
import pickle
import time
import random
host = socket.gethostbyname(socket.gethostname())
port = 12345
s = socket.socket()		    # TCP socket object
s.connect((host,port))
while True:
    res = [random.randrange(1, 50, 1) for i in range(7)]
    print(res)
    data=pickle.dumps(res)
    s.send(data)
    time.sleep(1)
    