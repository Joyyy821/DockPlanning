# -*- coding: utf-8 -*-
"""
Created on Sat Jul  2 15:25:57 2022

@author: 218019067
"""

import time
from matplotlib import pyplot as plt
import math
import numpy as np
import copy
import os

position = [] #{"x":[], "y":[]}
dock = [] #{"up":[], "down":[], "left":[], "right":[]}
dock_origin = []
globalMap = []

def getAllfile(root):
    data = os.listdir(root)
    folders = []
    for i in range(len(data)):
        if os.path.isdir(data[i]):
            folders.append(root + data[i] + "/")
    return folders

def readFile(file):
    global position 
    global dock
    global dock_origin
    with open(file, 'r') as f:
        for line in f:
            if "position" in line:
                p = eval(line[10:])
                position.append(p)
                
            if "dock" in line and "dock_origin" not in line:
                d = eval(line[6:])
                dock.append(d)
            if "dock_origin" in line:
                dock_origin = eval(line[13:])
    print("Original path: ", position)
    print("Dock state: ", dock)
    print("Initial dock joints distribution", dock_origin)
    
    
def pathGenerator():
    global position 
    global dock
    
    last_pos = copy.deepcopy(position[-1])
    i = 0
    while not position[i] == last_pos:
#        print(i)
#        print(position[i])
        step = 1
        if abs(position[i][0] - position[i+1][0]) > 1:
            step = insertPoint("x",i+1)
        elif abs(position[i][1] - position[i+1][1]) > 1:
            step = insertPoint("y",i+1)
        i += step   
        #time.sleep(1)
    print("Generate path: ", position)
    print("Dock state: ", dock)
    

def insertPoint(axis, index):
    global position 
    global dock
    
    if axis == "x":
        i = 0
        j = 1
    if axis == "y":
        i = 1
        j = 0
    
    inserted_lst_i = list(range(position[index-1][i], position[index][i], 
          np.sign(position[index][i]-position[index-1][i])))
    inserted_lst_i.pop(0)
    
    # another axis, copy last one
    inserted_lst_j = [position[index-1][j]] * len(inserted_lst_i)
    
    # insert the list
    count = index
    for x,y in zip(inserted_lst_i, inserted_lst_j):
        lst = [0,0]
        lst[i] = x
        lst[j] = y
        position.insert(count, lst)
        count += 1
    
    #print(position)
    
    # dock
    insert_dock_lst = [dock[index-1]]* len(inserted_lst_i)
    
    num = index
    for lst in insert_dock_lst:
        dock.insert(num, lst)
        num += 1
    
    #print(dock)
    # if new element no equal, insert
    # if equal, return false
    return len(inserted_lst_i)
    

def scaleMap(h, w):
    global position 
    global globalMap
    #       
    grid_size = 16
    h_unit = h/grid_size # height
    w_unit = w/grid_size # width
    
    for p in position:
        globalMap.append([p[1]*h_unit-h/2, p[0]*w_unit-w/2, p[2]])
    print("Global map: ", globalMap)
    

def writeFile(fileName):
    global globalMap
    global dock
    global dock_origin
    with open(fileName,'w',encoding='utf-8')as f:
        f.write("dock_origin: "+str(dock_origin)+"\n")
        for p,d in zip(globalMap, dock):
            f.write("position: " + str(p) + "\n" + "dock: " + str(d) + "\n")
    f.close()

if __name__ == "__main__":
    ratio = 4   # 24 grid -> 6m
    map_size = [16, 16]
    # file path
    root = "./"
    filepath = getAllfile(root)
    print(filepath)

    for i in range(len(filepath)):
    # for i in range(1):
        dire = filepath[i]
        for num in range(1,5):
            file = dire + str(num) + ".txt"
            readFile(file)
            pathGenerator()
            scaleMap(map_size[0]/ratio, map_size[1]/ratio)
            newfile = dire + str(num) + "_.txt"
            writeFile(newfile)
            position.clear()
            dock.clear() 
            globalMap.clear()
    
    