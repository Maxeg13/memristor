# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
from matplotlib import pyplot as plt
import matplotlib 
matplotlib.rcParams.update({'font.size': 8})
fig, axes = plt.subplots(5, 5)

# a=np.zeros((1, 3), dtype=int);
cnt = 0
filepath = 'C:/Users/DrPepper/Documents/memristor/analyze.txt'

with open(filepath) as fp:
   line = fp.readline()   
   while line:
       cnt += 1
       line = fp.readline()

a=np.zeros((cnt, 3), dtype=int);       
       

cnt = 0
with open(filepath) as fp:
   line = fp.readline()   
   while line:
       # print("Line {}: {}".format(cnt, line.strip()))
       a[cnt,:] = line.split("   ")       
       line = fp.readline()
       cnt += 1


for i in range(0,5):
    for j in range(0,5):
        # axes[i,j].xlabel('current,uA');
        axes[i,j].hist(a[(a[:,0]==i)&(a[:,1]==j),2])
        