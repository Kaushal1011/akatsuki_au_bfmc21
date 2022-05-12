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
    data['x'] = random.randrange(1, 50, 1)
    data['y'] = random.randrange(1, 50, 1)
    data['yaw'] = random.randrange(1, 50, 1)
    data['v'] = random.randrange(1, 30, 1)
    data['pitch'] = random.randrange(1, 50, 1)
    data['roll'] = random.randrange(1, 50, 1)
    data['rear_x'] = random.randrange(1, 50, 1)
    data['rear_y'] = random.randrange(1, 50, 1)
    data['target_x'] = random.randrange(1, 50, 1)
    data['target_y'] = random.randrange(1, 50, 1)
    data['target_idx'] = random.randrange(1, 50, 1)
    data['steering_angle'] = random.randrange(1, 50, 1)
    data['lk_angle'] = random.randrange(1, 50, 1)
    data['cs_angle'] = random.randrange(1, 50, 1)
    data['front_distance'] = random.randrange(1, 50, 1)
    data['side_distance'] = random.randrange(1, 50, 1)
    data['current_ptype'] = random.randrange(1, 50, 1)
    data['current_target'] = random.randrange(1, 50, 1)
    data['detected_intersection'] = bool(random.getrandbits(1))
    data['car_detected'] = bool(random.getrandbits(1))
    data['active_behaviours'] = random.randrange(1, 50, 1)
    data['roadblock'] = bool(random.getrandbits(1))
    data['pedestrian'] = bool(random.getrandbits(1))
    data['stop'] = bool(random.getrandbits(1))
    data['crosswalk'] = bool(random.getrandbits(1))
    data['highway_entry'] = bool(random.getrandbits(1))
    data['highway_exit'] = bool(random.getrandbits(1))
    data['priority'] = bool(random.getrandbits(1))
    data['onewayroad'] = bool(random.getrandbits(1))
    data['parking'] = bool(random.getrandbits(1))
    data['no_entry'] = bool(random.getrandbits(1))
    data['roundabout'] = bool(random.getrandbits(1))
    data = json.dumps(data)
    print(data)
    sock.sendto(data.encode('utf-8'), addr)
    # sock.sendall(bytes(data,encoding="utf-8"), addr)
    time.sleep(0.1)
    cnt+=1
    if cnt==650:
        break
    