import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#functions
def moving_avg(numpy_array):
    output = np.zeros(len(z),dtype='float')
    for i in range(len(numpy_array)):
        output[i] = (numpy_array[i-1]*0.25+numpy_array[i]*.50+numpy_array[i-2]*.1225+numpy_array[i-3]*0.1225)/3*9.8/8192

    return output

#show xy plots
df = pd.read_csv('dd2.csv')
print(df)
# plt.plot(df.index,df.x)
# plt.plot(df.index,df.y)
# plt.show()

#to numpy arrays
x = np.array(df.x)
y = np.array(df.y)
z = np.array(df.z)


#take z no grav
z_nograv = np.zeros(len(z),dtype='float')
for i in range(len(z)):
    z_nograv[i] = 8192+z[i]

p1=plt.figure(1)
plt.plot(z_nograv,label='z')
plt.plot(x,label='x')
plt.plot(y,label='y')
plt.legend()

x = moving_avg(x)
y = moving_avg(y)
z_nograv = moving_avg(z_nograv)

p2=plt.figure(2)
plt.plot(z_nograv,label='z')
plt.plot(x,label='x')
plt.plot(y,label='y')
plt.legend()

plt.show()
pass