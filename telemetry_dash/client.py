import socket
import json
import time
import random
host = socket.gethostbyname(socket.gethostname())
port = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)	    # TCP socket object
addr = (host, port)
sock.connect((host,port))

cnt=1
while True:
    data = {}
    data['X-Coord'] = random.randrange(1, 50, 1)
    data['Y-Coord'] = random.randrange(1, 50, 1)
    data['Yaw'] = random.randrange(1, 50, 1)
    data['Speed'] = random.randrange(1, 50, 1)
    data['Steering_Angle'] = random.randrange(1, 50, 1)
    data['Current_Behaviour'] = random.randrange(1, 50, 1)
    data['Detection'] = random.randrange(1, 50, 1)
    data = json.dumps(data)
    print(data)
    sock.sendto(data.encode('utf-8'), addr)
    # sock.sendall(bytes(data,encoding="utf-8"), addr)
    time.sleep(0.1)
    cnt+=1
    if cnt==650:
        break
    