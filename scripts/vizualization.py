#!/usr/bin/env python
import sys, os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pylab
from scipy.interpolate import interp1d
import time

fname = '1486168125011874907'
x = np.load('laser/'+fname+'.npy')
laser_angle_range = 1081*0.25
laser_angle = np.linspace(-1./2*laser_angle_range,1./2*laser_angle_range,1081, endpoint=True)
f = interp1d(laser_angle, x[::-1],kind='cubic')
# plt.plot(angle, x[::-1],'o')
x = np.load('depth/'+fname+'.npy')
x = np.mean(x,axis=0)


angle_range = 58
angle = np.linspace(-1./2*angle_range,1./2*angle_range,640, endpoint=True)

plt.plot(angle, f(angle),'ro')
plt.plot(angle,x,'go')
print(x.shape)
print(f(angle).shape)
# plt.plot(angle,f(angle)-x, 'o')
pylab.xlim([-1./2*angle_range, 1./2*angle_range])
pylab.ylim([0, 10])
plt.show()

directory = 'laser/'
out_directory = 'laser_fov/'
for filename in os.listdir(directory):
    if filename.endswith(".npy"):
        # print(os.path.join(directory, filename))
        print(filename)
        depth_in = np.load(directory+filename)
        f = interp1d(laser_angle, depth_in[::-1],kind='linear')
        data_out = f(angle)
        np.save(out_directory+filename,data_out)
        continue
    else:
        continue
