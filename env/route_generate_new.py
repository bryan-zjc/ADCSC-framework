# -*- coding: utf-8 -*-
"""
Created on Tue Mar  1 20:27:36 2022

@author: Administrator
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys  # 导入sys模块
#sys.setrecursionlimit(10**5)  # 将默认的递归深度修改为3000
import numpy as np
import random
import pandas as pd
from numpy import nan
import copy



incoming_link = ['1','6',
                 '-3','-8',
                 '21','26',
                 '-23','-28'
                 ]
outcoming_link = [str(-int(item)) for item in incoming_link
                 ]

od_num = len(incoming_link) * (len(incoming_link)-1)

demand_per_od = np.zeros(od_num)
for i in range(od_num):
    demand_per_od[i] = 400*len(incoming_link)/od_num


od_list = []
for m in range(len(incoming_link)):
    for n in range(len(outcoming_link)):
        if m != n:
            from_link = incoming_link[m]
            to_link = outcoming_link[n]
            od_list.append(str(from_link)+ ' ' + str(to_link))



#生成trip文件，需要再利用cmd命令转化为rou文件
def generate_tripfile(incoming_link,outcoming_link,demand_per_od):
    random.seed(37)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions    
    
    #修改换道属性在vType里面
    for s in range(1):
        filename=('Demand.trip.xml')
        with open(filename, "w") as routes:
            print("""<routes>
        <vType id="cav" accel="2" decel="4.5" tau="1.0" speedFactor="1.0" speedDev="0.0" sigma="0.5" length="7" minGap="3" maxSpeed="60" guiShape="passenger"/>
            """, file=routes)
            
            number = 0
            for m in range(len(incoming_link)):
                for n in range(len(outcoming_link)):
                    from_link = incoming_link[m]
                    to_link = outcoming_link[n]
                    alpha = 1
                    if from_link == '-219' or from_link =='-225'or from_link =='-230'or from_link =='-235':
                        alpha = 0.6
                    
                    print('''
        <flow id="OD%i" begin="0" end= "3600" vehsPerHour="%s" type='cav' from="%s" to="%s"/>''' % (number, demand_per_od[m]*alpha,from_link, to_link), file=routes)
                    number += 1

            print("</routes>", file=routes)


generate_tripfile(incoming_link,outcoming_link,demand_per_od)

#duarouter --route-files=Demand.trip.xml --net-file=2x2_demo.net.xml --output-file=Demand.rou.xml --weights.random-factor=1000