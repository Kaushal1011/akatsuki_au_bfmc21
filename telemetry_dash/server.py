import socket
import pandas as pd
import pickle
host = socket.gethostbyname(socket.gethostname())
port = 12345
s = socket.socket()		    # TCP socket object
s.bind((host,port))

s.listen(1)

conn, addr = s.accept()
print("server is on")
df = pd.DataFrame(columns = ['X-Coord', 'Y-Coord', 'Yaw', 'Speed', 'Steering_Angle', 'Current_Behaviour', 'Detection'])
print ("Connected by ", addr)
while True:
    recvd=conn.recv(4096)
    data = pickle.loads(recvd)
    print(data)
    # d = data.split(",")
    df.loc[len(df)] = data
    df.to_csv('tele.csv',mode='w+')
    