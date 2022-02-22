import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
import math


#replace this with name of file you want to use
def log2csv_rename():
    rm = 'data/sensvals.log'
    ds = 'data/sensvals'
    end = '.csv'

    #remove putty header
    file = open(rm,'r')
    ar = file.readlines()
    file.close()
    del ar[0]
    now = datetime.now()
    now = now.strftime("%d-%m-%Y-%H-%M")
    file = open(ds+now+end,'w+')
    file.write('ax,ay,az,gx,gy,gz\n')
    for i in range(len(ar)):
        file.write(ar[i])
    file.close()
    return

def accel_mpers(df,sensitivity):

    for ind,row in enumerate(df):
        for i,elem in enumerate(row):
            df[ind,i] = (elem/sensitivity*980) #cm/s^2
    return df

def drop_gyro(df):
    df = df.drop(['gx', 'gy', 'gz'],axis='columns')
    df = df.to_numpy()
    return df

def drop_accel(df):
    df = df.drop(['ax','ay','az'],axis='columns')
    df = df.to_numpy()
    return df

def accel_angles(df):
    pitchAcc = np.zeros(df.shape[0])
    rollAcc = np.zeros(df.shape[0])
    i=0
    for row in df:
        pitchAcc[i] = math.atan2(row['y'],row['z']) * 180/math.pi
        rollAcc[i] = math.atan2(row['x'],row['z'])*180/math.pi
        i+=1
    rot = np.concatenate((pitchAcc,rollAcc),axis=1)
    return rot

def gyro_deg(df,sensitivity):
    for ind,row in enumerate(df):
        for i,elem in enumerate(row):
            df[ind,i] = (elem/sensitivity)*6 #deg/s
    return df

def gyro_integ(df):
    dt = 0.05 #50 ms sample rate
    pitchGy = np.zeros(df.shape[0],dtype='int64')
    rollGy = np.zeros(df.shape[0],dtype='int64')

    
    for i,row in enumerate(df):
        if i != 0:
            pitchGy[i] = pitchGy[i-1] + ((row[0]) * dt) #x
            rollGy[i] = rollGy[i-1] + ((row[1]) * dt) #y
        else:
            pitchGy[i] = row[0]  * dt
            rollGy[i] = row[1] * dt

    rot = np.concatenate((np.transpose(pitchGy),np.transpose(rollGy)))
    return rot

def complementaryFilter(rotg,rota):
    pitchcompl = np.zeros(len(rotg.shape[0]))
    rollcompl = np.zeros(len(rotg.shape[0]))
    df = np.concatenate((rota,rotg),axis=1) # [pitcha,rolla,pitchg,rollg]
    i=0
    for row in df:
        pitchcompl[i] = row[2] * 0.98 + row[0] * 0.02
        rollcompl[i] = row[3] * 0.98 + row[1] * 0.02
        i+=1

    rot = np.concatenate((pitchcompl,rollcompl),axis=1)
    return rot #[pitch,roll]


