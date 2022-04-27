import pandas as pd
 
# intialise data of lists.
data = {'X-Coord':[0,1,2,3],
        'Y-Coord':[0,1,2,3],
        'Yaw':[20,30,45,60],
        'Speed':[10,12,8,16],
        'Steering_Angle':[20,10,0,14],
        'Current_Behaviour':['lk', 'park', 'cs', 'cs'],
        'Detection':['stop', 'no sign', 'no sign', 'priority']}
 
# Create DataFrame
df = pd.DataFrame(data)
df.to_csv('tele.csv',mode='w+')
 
# Print the output.
print(df)