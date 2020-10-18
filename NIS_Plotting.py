import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
#matplotlib inline

print(os.getcwd())

data_r = pd.read_csv('./build/Radar_NIS_more_pnoise.txt', sep=",", usecols=[0])
data_r.head()
fig1 = plt.figure(figsize=(10,6))
plt.plot(data_r, label='NIS Radar Data')
plt.plot([0,len(data_r)],[7.815,7.815],'r--',lw=2, label='Chi square = 7.815 for 3 DOF')
print(len(data_r))
plt.xlabel('x')
plt.ylabel('y')
plt.title('NIS Radar vs Steps')
plt.legend()
plt.savefig('Output_Images/NIS_Radar_more_pnoise.png')
plt.show()


data_l = pd.read_csv('./build/Lidar_NIS_more_pnoise.txt', sep=",", usecols=[0])
data_l.head()
plt.figure(figsize=(10,6))
plt.plot(data_l, label='NIS Lidar Data')
plt.plot([0,len(data_l)],[5.991,5.991],'r--',lw=2, label='Chi square = 5.991 for 2 DOF')
print(len(data_l))
plt.xlabel('x')
plt.ylabel('y')
plt.title('NIS Lidar vs Steps')
plt.legend()
plt.savefig('Output_Images/NIS_Lidar_more_pnoise.png')
plt.show()
