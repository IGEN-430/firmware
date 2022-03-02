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
            df[ind,i] = round(elem/sensitivity,2) #gs
    return df

def drop_gyro(df):
    df = df.drop(['gx', 'gy', 'gz'],axis='columns')
    df = df.to_numpy(dtype='float')
    return df

def drop_accel(df):
    df = df.drop(['ax','ay','az'],axis='columns')
    df = df.to_numpy(dtype='float')
    return df

def accel_angles(df):
    pitchAcc = np.zeros(df.shape[0])
    rollAcc = np.zeros(df.shape[0])
    z = np.zeros(df.shape[0])

    for i,row in enumerate(df):
        rollAcc[i] = row[1]*90 # around x axis -- therefore measuring the y axis compared to g
        pitchAcc[i] = row[0]*90
        # rollAcc[i] = math.atan2(row[1],row[2])*180/math.pi
        # pitchAcc[i] = math.atan2(-row[0],math.sqrt(math.pow(row[2],2)+math.pow(row[1],2)))*180/math.pi

    rot = np.array([(rollAcc),(pitchAcc)])
    rot = np.transpose(rot)
    return rot

def gyro_deg(df,sensitivity):
    for ind,row in enumerate(df):
        for i,elem in enumerate(row):
            df[ind,i] = elem/sensitivity*90/math.pi #deg/s
    return df

def gyro_integ(df):
    dt = 0.135 #50 ms sample rate
    pitchGy = np.zeros(df.shape[0],dtype='float64')
    rollGy = np.zeros(df.shape[0],dtype='float64')

    
    for i,row in enumerate(df):
        if i != 0:
            rollGy[i] = rollGy[i-1] + ((row[0]) * dt) #x
            pitchGy[i] = pitchGy[i-1] - ((row[1]) * dt) #y
        else:
            rollGy[i] = row[0]  * dt
            pitchGy[i] = -row[1] * dt

    rot = np.array([rollGy,pitchGy])
    rot = np.transpose(rot)
    return rot

def complementaryFilter(rotg,rota):
    pitchcompl = np.zeros(rotg.shape[0])
    rollcompl = np.zeros(rotg.shape[0])
    df = np.concatenate((rota,rotg),axis=1) # [rolla,pitcha,rollg,pitchg]
    # df = np.transpose(df)
    i=0
    for row in df:
        rollcompl[i] = row[2] * 0.1 + row[0] * 0.9
        pitchcompl[i] = row[3] * 0.1 + row[1] * 0.9
        i+=1

    rot = np.array([rollcompl,pitchcompl])
    rot = np.transpose(rot)
    return rot 
