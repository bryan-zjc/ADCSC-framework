# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 13:56:25 2024

@author: Jichen Zhu
"""

import numpy as np

# intersection id
inters_list = [0,1,4,5]


# approach link id of each inter
link = [['1','-2','21','-22'],
        ['2','-3','26','-27'],
        
        ['6','-7','22','-23'],
        ['7','-8','27','-28'],
       ]

# adjacent inters ids
neighbors = [['1','4'],
             ['0','5'],
             
             ['0','5'],
             ['1','4'],
            ]

i_number = 4 #number of inters

link_type = np.zeros([i_number,8]) #1-source link, 0-connecting link
link_type[0][0] = link_type[0][2] = link_type[0][4] = link_type[0][6] = 1
link_type[1][4] = link_type[1][6] = link_type[1][1] = link_type[1][3] = 1
link_type[2][0] = link_type[2][2] = link_type[2][5] = link_type[2][7] = 1
link_type[3][1] = link_type[3][3] = link_type[3][5] = link_type[3][7] = 1


up_inter = np.zeros([i_number,8,1])-1 #Store the upstream intersection id of each entrance lane m at each intersection i
up_movement = np.zeros([i_number,8,2])-1 #Store the entrance lane id of each upstream intersection of each entrance lane m at intersection i
up_inter[0,1,] = 1
up_inter[0,3,] = 1
up_inter[0,5,] = 4
up_inter[0,7,] = 4
up_movement[0,1,] = [3,5]
up_movement[0,3,] = [3,5]
up_movement[0,5,] = [0,7]
up_movement[0,7,] = [0,7]

up_inter[1,0,] = 0
up_inter[1,2,] = 0
up_inter[1,5,] = 5
up_inter[1,7,] = 5
up_movement[1,0,] = [2,4]
up_movement[1,2,] = [2,4]
up_movement[1,5,] = [0,7]
up_movement[1,7,] = [0,7]

up_inter[2,1,] = 5
up_inter[2,3,] = 5
up_inter[2,4,] = 0
up_inter[2,6,] = 0
up_movement[2,1,] = [3,5]
up_movement[2,3,] = [3,5]
up_movement[2,4,] = [1,6]
up_movement[2,6,] = [1,6]

up_inter[3,0,] = 4
up_inter[3,2,] = 4
up_inter[3,4,] = 1
up_inter[3,6,] = 1
up_movement[3,0,] = [2,4]
up_movement[3,2,] = [2,4]
up_movement[3,4,] = [1,6]
up_movement[3,6,] = [1,6]



down_inter = np.zeros([i_number,8,1])-1
down_movement = np.zeros([i_number,8,2])-1
down_inter[0,1,] = 4
down_inter[0,6,] = 4
down_inter[0,2,] = 1
down_inter[0,4,] = 1
down_movement[0,1,] = [4,6]
down_movement[0,6,] = [4,6]
down_movement[0,2,] = [0,2]
down_movement[0,4,] = [0,2]

down_inter[1,1,] = 5
down_inter[1,6,] = 5
down_inter[1,3,] = 0
down_inter[1,5,] = 0
down_movement[1,1,] = [4,6]
down_movement[1,6,] = [4,6]
down_movement[1,3,] = [1,3]
down_movement[1,5,] = [1,3]

down_inter[2,0,] = 0
down_inter[2,7,] = 0
down_inter[2,2,] = 5
down_inter[2,4,] = 5
down_movement[2,0,] = [5,7]
down_movement[2,7,] = [5,7]
down_movement[2,2,] = [0,2]
down_movement[2,4,] = [0,2]

down_inter[3,0,] = 1
down_inter[3,7,] = 1
down_inter[3,3,] = 4
down_inter[3,5,] = 4
down_movement[3,0,] = [5,7]
down_movement[3,7,] = [5,7]
down_movement[3,3,] = [1,3]
down_movement[3,5,] = [1,3]



allowed_movements1 = [[0],
                     [1],
                     [2],
                     [3],
                     [4],
                     [5],
                     [6],
                     [7],
                     [0,1],
                     [0,2],
                     [0,7],
                     [1,3],
                     [1,6],
                     [2,3],
                     [2,4],
                     [3,5],
                     [4,5],
                     [4,6],
                     [5,7],
                     [6,7]    
                    ] #allowed phase group

allowed_movements2 = [[0],
                     [1],
                     [2],
                     [3],
                     [4],
                     [5],
                     [6],
                     [7],
                     [1,0],
                     [2,0],
                     [7,0],
                     [3,1],
                     [6,1],
                     [3,2],
                     [4,2],
                     [5,3],
                     [5,4],
                     [6,4],
                     [7,5],
                     [7,6]    
                    ] #allowed phase group

allowed_phase = ["GrrGrrGrrGrG",
                 "GrrGrGGrrGrr",
                 "GrrGrrGrrGGr",
                 "GrrGGrGrrGrr",
                 "GrGGrrGrrGrr",
                 "GrrGrrGrGGrr",
                 "GGrGrrGrrGrr",
                 "GrrGrrGGrGrr",
                 "GrrGrGGrrGrG",
                 "GrrGrrGrrGGG",
                 "GrrGrrGGrGrG",
                 "GrrGGGGrrGrr",
                 "GGrGrGGrrGrr",
                 "GrrGGrGrrGGr",
                 "GrGGrrGrrGGr",
                 "GrrGGrGrGGrr",
                 "GrGGrrGrGGrr",
                 "GGGGrrGrrGrr",
                 "GrrGrrGGGGrr",
                 "GGrGrrGGrGrr"    
                 ] #allowed phase group in SUMO

all_red_phase = "rrrrrrrrrrrr"



directions = ['WE/L','WE/T','NS/L','NS/T'] 

# parameters
arrival = 400/3600/3
vf = 50/3.6 # free flow speed（m/s）
rolling_horizon = int(300/vf) #The rolling horizon step, in this demo all links are 300m.
g_min = 10
g_max = 40
T = rolling_horizon #planing horizon
p = 1/3 #turning ratio
saturation = 1500/3600 #saturation flow/departure flow
delta_t = 1
gamma = np.ones([8,8])
gamma[0][1] = gamma[0][2] = gamma[0][7] = gamma[1][3] = gamma[1][6] = gamma[2][3] = gamma[2][4] = gamma[3][5] = gamma[4][5] = gamma[4][6] = gamma[5][7] = gamma[6][7] = 0
gamma[1][0] = gamma[2][0] = gamma[7][0] = gamma[3][1] = gamma[6][1] = gamma[3][2] = gamma[4][2] = gamma[5][3] = gamma[5][4] = gamma[6][4] = gamma[7][5] = gamma[7][6] = 0

pi = 2*gamma #inter green time
M = 100000

update_win = np.zeros(i_number)+rolling_horizon