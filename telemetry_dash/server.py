from email import header
import socket
from cv2 import DFT_REAL_OUTPUT
from numpy import true_divide
import pandas as pd
from json import loads
import json
import csv

host = socket.gethostbyname(socket.gethostname())
port = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host,port))
cnt=1
final_data={}
while True:
    data, addr = sock.recvfrom(4096)
    data = data.decode('utf-8')
    dictData=loads(data)
    with open("sample.json", "w") as outfile:
        json.dump(dictData, outfile)
    print(dictData)
    print(cnt)
    cnt+=1
    # df=pd.DataFrame()
    # df.loc[len(df)] = dictData
    # df_tail_1 = df.tail(1)
    # print(df_tail_1)
    # df_tail_1.to_csv('tele.csv', mode='a', header=False)

    