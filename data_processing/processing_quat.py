import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def quat_2_euler(w,x,y,z):
    roll_x = np.zeros(len(x))
    pitch_y = np.zeros(len(y))
    yaw_z = np.zeros(len(z))
    for i in range(len(x)):
        roll_x[i] = math.degrees(math.atan2(2*x[i]*y[i]-2*w[i]*z[i],2*w[i]*w[i]+2*x[i]*x[i]-1))
        pitch_y[i] = math.degrees(-math.asin(2*x[i]*z[i]+2*w[i]*y[i]))
        yaw_z[i] = math.degrees(math.atan2(2*y[i]*z[i]-2*w[i]*x[i],2*w[i]*w[i]+2*z[i]*z[i]-1))
    return roll_x,pitch_y,yaw_z

df = pd.read_csv("dd5.csv")

w = np.array(df.w)
x = np.array(df.x)
y = np.array(df.y)
z = np.array(df.z)

roll,pitch,yaw = quat_2_euler(w,x,y,z)

plt.figure(1)
plt.plot(roll,label='roll_x')
plt.plot(pitch,label='pitch_y')
plt.plot(yaw,label='yaw_z')
plt.legend()
plt.show()
pass
