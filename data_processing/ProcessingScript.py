from Processing import *
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

## Script - Rename my log to csv file!
# log2csv_rename()

# Line to read data from fiel
df = pd.read_csv('data/sensvals24-02.csv') # always change!!!!!!!!!!!!

## Script - Plot my raw data values and save to output file
# df.plot(y=['ax','ay','az','gx','gy','gz'],title='Moving Test',xlabel='time',ylabel='sensorvalue')
# plt.savefig('results/raw-moving.png') #always change!!!!!!!!!!!!!!!!

## Script - Plot accel values in cm/s^2 and gyro values in deg/s
df1 = drop_gyro(df)
df1 = accel_mpers(df1,8192)
rot_accel = accel_angles(df1)

df2 = drop_accel(df)
df2 = gyro_deg(df2,131)
rot_gyro = gyro_integ(df2)

rot_c = complementaryFilter(rot_gyro,rot_accel)

## Script for plotting linear acceleration in cm/s^2
# plt.figure()
# plt.plot(df1[:,0],label='accel_x')
# plt.plot(df1[:,1],label='accel_y')
# plt.plot(df1[:,2],label='accel_z')
# plt.xlabel('time')
# plt.ylabel('cm/s^2')
# plt.title('acceleration raw values in cm/s^2')
# plt.legend()

# plt.figure()
# plt.plot(rot_gyro[:,0],label='gyro_angle')
# plt.plot(rot_accel[:,0],label='accel_angle')
# plt.plot(rot_c[:,0],label='comp_filter')
# plt.xlabel('time')
# plt.ylabel('deg')
# plt.title('Comparison of pitch around Xaxis angles calculated')
# plt.legend()

# plt.figure()
# plt.plot(rot_gyro[:,1],label='gyro_angle')
# plt.plot(rot_accel[:,1],label='accel_angle')
# plt.plot(rot_c[:,1],label='comp_filter')
# plt.xlabel('time')
# plt.ylabel('deg')
# plt.title('Comparison of roll around Yaxis angles calculated')
# plt.legend()

# plt.show()

dfg = pd.DataFrame(rot_gyro)
dfg.columns = ['rollx','pitchy']
dfa = pd.DataFrame(rot_accel)
dfa.columns = ['rollx','pitchy']
dfc = pd.DataFrame(rot_c)
dfc.columns = ['rollx','pitchy']

dfroll = pd.concat([dfg['pitchy'],dfa['pitchy'],dfc['pitchy']],axis=1,join='inner')
dfroll.columns = ['gyroscope_raw','accelerometer_raw','complimentary_filter']
sns.set_theme()
p = sns.lineplot(data=dfroll)
p.set_title('Analysis of data')
p.set_xlabel('Time (bins)')
p.set_ylabel('Angle (degrees)')

plt.show()
print("hi")