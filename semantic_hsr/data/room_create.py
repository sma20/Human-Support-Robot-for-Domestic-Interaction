#!/usr/bin/python
# -*- coding: utf-8 -*-

import csv
import numpy
import pandas as pd
M=[[0 for j in range(0,5)] for i in range(0,6)]
M=[['room','xmin','xmax','ymin','ymax'], ['home',-1,8,-3.2,3.5],['corridor',-1, 0.5,-3.2,0], ['kitchen',0.5,5,0,3.5], ['bathroom',0.5,3.1,-3.2,0], ['bedroom',3.1,8,-3.2,0], ['living_room',5,8,0,3.5]]

data=pd.DataFrame(M)
data.to_csv('room.csv', index=False, header=False)


#brieuc... you forgot the home coordinates......... i added it 
