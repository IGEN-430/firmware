import numpy as np
from math import pi
from scipy import signal

def difference_eq():
    w0 = 8 #hertz  ==================+EDIT THIS
    w0 = w0*2*pi #rad/s
    denom = [1,w0]
    fs = 100 #hz

    lowpass = signal.TransferFunction(w0,denom,dt=1/fs)
    print(lowpass)
    print('X coefficients\t'+str(lowpass.num))
    print('Y coefficients\t'+str(lowpass.den))

difference_eq()