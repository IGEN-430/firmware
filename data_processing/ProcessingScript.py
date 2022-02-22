from Processing import *
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

## Script - Rename my log to csv file!
# log2csv_rename()

# Line to read data from fiel
df = pd.read_csv('data/sensvals-moving-21-02.csv') # always change!!!!!!!!!!!!

## Script - Plot my raw data values and save to output file
# df.plot(y=['ax','ay','az','gx','gy','gz'],title='Moving Test',xlabel='time',ylabel='sensorvalue')
# plt.savefig('results/raw-moving.png') #always change!!!!!!!!!!!!!!!!

## Script - Plot accel values in cm/s^2 and gyro values in deg/s
df1 = drop_gyro(df)
df1 = accel_mpers(df1,4096)

df2 = drop_accel(df)
df2 = gyro_deg(df2,131)
rot_gyro = gyro_integ(df2)

plt.figure()
plt.plot(df1[:,0],label='accel_x')
plt.plot(df1[:,1],label='accel_y')
plt.plot(df1[:,2],label='accel_z')
plt.xlabel('time')
plt.ylabel('cm/s^2')
plt.title('acceleration raw values in cm/s^2')
plt.legend()

plt.figure()
plt.plot(df2[:,0],label='pitch_x')
plt.plot(df2[:,1],label='roll_y')
plt.xlabel('time')
plt.ylabel('deg')
plt.title('tilt position (gyro only in x and y)')
plt.legend()

plt.show()