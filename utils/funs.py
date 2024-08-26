# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 14:05:10 2024

@author: Jichen Zhu
"""

import numpy as np
import traci
import gurobipy
from utils import params


inters_list = params.inters_list
link = params.link
i_number = params.i_number
neighbors = params.neighbors
link_type = params.link_type
up_inter = params.up_inter
up_movement = params.up_movement
down_inter = params.down_inter
down_movement = params.down_movement
allowed_movements1 = params.allowed_movements1
allowed_movements2 = params.allowed_movements2
allowed_phase = params.allowed_phase
all_red_phase = params.all_red_phase
directions = params.directions
arrival = params.arrival
vf = params.vf
rolling_horizon = params.rolling_horizon
g_min = params.g_min
g_max = params.g_max
T = params.T
p = params.p
saturation = params.saturation
delta_t = params.delta_t
gamma = params.gamma
pi = params.pi
M = params.M
update_win = params.update_win


def get_vehicle_number(i,direction,link):
    links = link[i]
    if direction == 'WE/T':
       t_link = links[0:2]
       lane_number = []
       for link_name in t_link:
           lane_name = link_name+'_'+str(1)
           lane_number.append(traci.lane.getLastStepVehicleNumber(lane_name))                              
    
    if direction == 'NS/T':
       t_link = links[2:4]
       lane_number = []
       for link_name in t_link:           
           lane_name = link_name+'_'+str(1)
           lane_number.append(traci.lane.getLastStepVehicleNumber(lane_name))

    if direction == 'WE/L':
       t_link = links[0:2]
       lane_number = []
       for link_name in t_link:
           lane_name = link_name+'_'+str(2)
           lane_number.append(traci.lane.getLastStepVehicleNumber(lane_name))
    
    if direction == 'NS/L':
       t_link = links[2:4]
       lane_number = []
       for link_name in t_link:           
           lane_name = link_name+'_'+str(2)
           lane_number.append(traci.lane.getLastStepVehicleNumber(lane_name))

    return lane_number 


def get_initial_queue():
    q0 = np.zeros([i_number,8])
    for i in range(i_number):
        for d in range(4):
            direction = directions[d]
            q0[i,2*d] = get_vehicle_number(i,direction,link)[0]
            q0[i,2*d+1] = get_vehicle_number(i,direction,link)[1]
    return q0


def get_vehicle_out_signal(s,e,i):
    out = np.zeros([8,int(update_win[i])])
    for m in range(8):
        out[m,int(s[i,m]):int(e[i,m]+1)] = saturation*delta_t
    return out 


def get_vehicle_out(s,e,q_total_hi,i):
    v_out = np.zeros([i_number,8,int(update_win[i])])
    out = get_vehicle_out_signal(s,e,i)

    for m in range(8):
        for t in range(update_win[i]):
            v_out[i,m,t] = min(out[i,m,t],q_total_hi[m,t])
                
    return v_out 


def get_vehicle_in(s,e,q_total_hi,i):
    v_out = get_vehicle_out(s,e,q_total_hi,i)
    v_in = np.zeros([8,int(update_win[i])])
    for i in range(i_number):
        for m in range(8):
            if link_type[i,m] == 1:
                if link_type[i,m] == 1:
                    v_in[i,m,] = arrival
            else:
                up_inter_id = inters_list.index(up_inter[i,m])
                up_movement_id = up_movement[i,m,]
                v_in[i,m,] = (v_out[int(up_inter_id),int(up_movement_id[0])] + 
                              v_out[int(up_inter_id),int(up_movement_id[1])])*p
    return v_in 


s_total, e_total, eta_total = [], [], []
def optimization_signal(i,v_in,q_start_opt,C): #signal optimization for intersection i
    q_in = v_in[i,]
    q0_total = q_start_opt
    q0 = q0_total[i]
    q_d_0 = np.zeros(8)
    for m in range(8):
        if down_inter[i,m,] != -1:
            down_inter_id = inters_list.index(down_inter[i,m,])
            down_movement_id = down_movement[i,m,]
            if m in [0,7]:
                index = 0
                q_d_0[2*index] = q0_total[int(down_inter_id),int(down_movement_id[0])] 
                q_d_0[2*index+1] = q0_total[int(down_inter_id),int(down_movement_id[1])]    

            if m in [1,6]:
                index = 0
                q_d_0[2*index] = q0_total[int(down_inter_id),int(down_movement_id[0])] 
                q_d_0[2*index+1] = q0_total[int(down_inter_id),int(down_movement_id[1])]               

            if m in [2,4]:
                index = 0
                q_d_0[2*index] = q0_total[int(down_inter_id),int(down_movement_id[0])]  
                q_d_0[2*index+1] = q0_total[int(down_inter_id),int(down_movement_id[1])]        

            if m in [3,5]:
                index = 0
                q_d_0[2*index] = q0_total[int(down_inter_id),int(down_movement_id[0])] 
                q_d_0[2*index+1] = q0_total[int(down_inter_id),int(down_movement_id[1])]  

    
    mdl = gurobipy.Model(name='signal_optimization_inter%i' % i)
    
    s1 = mdl.addVars(range(0,8),lb=-40,ub=T,vtype=gurobipy.GRB.CONTINUOUS, name='s1')
    e1 = mdl.addVars(range(0,8),lb=-40,ub=T,vtype=gurobipy.GRB.CONTINUOUS, name='e1')
    q_out = mdl.addVars(range(0,8*T),lb=0,ub=saturation*delta_t,vtype=gurobipy.GRB.CONTINUOUS,name='q_out')
    q = mdl.addVars(range(0,8*T),lb=0,ub=65,vtype=gurobipy.GRB.CONTINUOUS,name='lambda')
    y1 = mdl.addVars(range(0,8*T),vtype=gurobipy.GRB.BINARY, name='y1')
    y2 = mdl.addVars(range(0,8*T),vtype=gurobipy.GRB.BINARY, name='y2')
    y = mdl.addVars(range(0,8*T),vtype=gurobipy.GRB.BINARY, name='y')
    phi = mdl.addVars(range(0,8*8),vtype=gurobipy.GRB.BINARY, name='phi')
    eta = mdl.addVars(range(0,8),vtype=gurobipy.GRB.BINARY, name='eta')
    mdl.update()
    

    for m in range(8):
        for n in range(8):
            if m == n:
                mdl.addConstr(phi[8*m+n] == 0)
            else:
                mdl.addConstr(s1[n] >= e1[m] + pi[m][n] - M*(2-gamma[m][n]-phi[8*m+n]) - M*(2-eta[m]-eta[n]))

                mdl.addConstr(e1[n] <= s1[m] + pi[m][n] + M*(1-gamma[m][n]+phi[8*m+n]) + M*(2-eta[m]-eta[n]))
    for m in range(8):
        mdl.addConstr(e1[m] >= s1[m]+g_min*eta[m])
        mdl.addConstr(e1[m] <= s1[m]+g_min*eta[m]*M)

    for m in range(8):
        mdl.addConstr(e1[m] <= s1[m]+g_max)

    for m in range(8):
        for t in range(T):
            if t == 0:
                mdl.addConstr(q[8*t+m] == int(q0[m]))
            else:
                mdl.addConstr(q[8*t+m] == q[8*(t-1)+m] + q_in[m][t] - q_out[8*t+m])

    for m in range(8):
        for t in range(T):
            if t == 0:
                mdl.addGenConstrIndicator(y[8*t+m], 1, s1[m] == 0)
                mdl.addGenConstrIndicator(y[8*t+m],1, q_out[8*t+m] <= q[8*(t)+m])
                mdl.addGenConstrIndicator(y[8*t+m],0, q_out[8*t+m] == 0)
            
            else:
                mdl.addGenConstrIndicator(y1[8*t+m], 1, s1[m]-t <= 0)
                mdl.addGenConstrIndicator(y2[8*t+m], 1, e1[m]-t >= 0)
                mdl.addGenConstrAnd(y[8*t+m],[y1[8*t+m],y2[8*t+m]])
                mdl.addGenConstrIndicator(y[8*t+m],1, q_out[8*t+m] <= q[8*(t-1)+m])
                mdl.addGenConstrIndicator(y[8*t+m],0, q_out[8*t+m] == 0)
            
    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [0,7] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[0])
    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [0,7] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[1])

    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [1,6] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[2])
    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [1,6] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[3])

    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [2,4] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[4])
    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [2,4] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[5])
    
    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [3,5] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[6])
    mdl.addConstr(gurobipy.quicksum(q_out[8*t+m] for m in [3,5] for t in range(rolling_horizon))*p <= C[i,m] + saturation*rolling_horizon - q_d_0[7])
 
    # 信号方案保留约束
    if len(s_total) > 0 and len(e_total) > 0:
        e_last = e_total[-1]
        s_last = s_total[-1]
        eta_last = eta_total[-1]
        current_t = update_win[i]
        
        
        for m in range(8):
            if s_last[m] < current_t and e_last[m] >= current_t:
                mdl.addConstr(s1[m] == s_last[m]-T)
                mdl.addGenConstrIndicator(eta[m], 0, e1[m] == s1[m])
            else:
                mdl.addGenConstrIndicator(eta[m], 0, e1[m] == 0)
                mdl.addConstr(s1[m] >= 0)
            
            if e_last[m] <= current_t and eta_last[m] == 1 and e_last[m] > 0:
                for n in range(8):
                    if m == n:
                        pass
                    else:
                        mdl.addConstr(s1[n] >= (e_last[m]-current_t) + pi[m][n] - M*(1-gamma[m][n]) - M*(1-eta[n]))# 冲突相位之间的间隔
            
    mdl.setObjective(gurobipy.quicksum(q_out[8*t+m] for m in range(8) for t in range(T)) - 
                     gurobipy.quicksum(q[8*t+m]for m in range(8) for t in range(T)), 
                     sense=gurobipy.GRB.MAXIMIZE)

    mdl.setParam('OutputFlag', 0)
    mdl.setParam("Timelimit", 10)
    mdl.optimize()

    s = []
    e = []
    eta1 = []
    lambda_ = []
    
    for item in mdl.getVars():
        if 's1' in str(item.varName):
            s.append(round(item.x))
        elif 'e1' in str(item.varName):
            e.append(round(item.x))
        elif 'eta' in str(item.varName):
            eta1.append(int(item.x))
        elif 'lambda' in str(item.varName):
            lambda_.append(int(item.x))
    

    
    print('-----------------------------------')
    inter_id = inters_list[i]
    print('Intersection ' + str(inter_id))
    print('s:')
    print(s)
    print('e:')
    print(e)
    print('eta:')
    print(eta1)
    print('Soultion time ' + str(mdl.Runtime))
    
    e_total.append(e)
    s_total.append(s)
    eta_total.append(eta1)
    
    return s,e,mdl.Runtime


def find_all_index(arr,item):
    return [i for i,a in enumerate(arr) if a==item]


def get_signal_time(i,s,e,step):
    start_time = []
    end_time = []
    green_time = []
    control_m = []
    sort_green = (np.argsort(np.array(s)))
    for m in sort_green:
        if s[m] < 0 and e[m] > 0:
            s[m] = 0  
    
    for m in sort_green:
        if s[m] <= update_win[i]:
            if e[m] > s[m]:
                start_time.append(step+s[m])
                end_time.append(step+e[m])
                green_time.append(e[m]-s[m])
                control_m.append(m)
    return start_time, end_time, green_time, control_m


def text_save(filename, data):
  file = open(filename,'a')
  for i in range(len(data)):
    s = str(data[i]).replace('[','').replace(']','')
    s = s.replace("'",'').replace(',','') +'\n'
    file.write(s)
  file.close()    


def calculate_queue_dynamics(i_number, T, step, update_win, arrival, total_queue, start_time_total, end_time_total, link_type, up_inter, up_movement, travel_t, saturation, delta_t, p, rolling_horizon):
    q_current = np.zeros([i_number, 8, rolling_horizon])
    q_start_opt = np.zeros([i_number, 8])
    
    for i in range(i_number):
        opt_start_point = step + update_win[i]
        opt_end_point = opt_start_point + T
        q0 = get_initial_queue()

        q_predict1 = np.zeros([8, rolling_horizon])
        q_out1 = get_vehicle_out_signal(start_time_total[-1], end_time_total[-1], i)
        
        q_history_hi = np.zeros([i_number, 8, rolling_horizon])
        q_out_signal = np.zeros([i_number, 8, rolling_horizon])
        q_in_current = np.zeros([8, rolling_horizon])
        
        for m in range(8):
            if link_type[i, m] == 1:
                q_in_current[m, :] = arrival
            else:
                up_inter_id = inters_list.index(up_inter[i, m])
                up_movement_id = up_movement[i, m]
                for t in range(rolling_horizon):
                    q_history_hi[int(up_inter_id), int(up_movement_id[0]), t] = total_queue[int(step - travel_t[i, m] + t)][int(up_inter_id)][int(up_movement_id[0])]
                    q_history_hi[int(up_inter_id), int(up_movement_id[1]), t] = total_queue[int(step - travel_t[i, m] + t)][int(up_inter_id)][int(up_movement_id[1])]

                for t in range(rolling_horizon):
                    v_out0 = min(q_history_hi[int(up_inter_id), int(up_movement_id[0]), t], q_out_signal[int(up_inter_id), int(up_movement_id[0]), t])
                    v_out1 = min(q_history_hi[int(up_inter_id), int(up_movement_id[1]), t], q_out_signal[int(up_inter_id), int(up_movement_id[1]), t])
                    q_in_current[m, t] = (v_out0 + v_out1) * p
        
        for m in range(8):
            for t in range(rolling_horizon):
                if t == 0:
                    q_predict1[m, t] = max(q0[i, m] + q_in_current[m, t] - q_out1[m, t], 0)
                else:
                    q_predict1[m, t] = max(q_predict1[m, t - 1] + q_in_current[m, t] - q_out1[m, t], 0)
        
        q_start_opt[i, :] = q_predict1[:, -1]
        q_current[i, :, :] = q_predict1
    
    return q_current, q_start_opt


def calculate_opt_horizon(i_number, step, rolling_horizon, arrival, total_queue, start_time_total, end_time_total, link_type, up_inter, up_movement, travel_t, saturation, delta_t, p):
    q_in_opt_horizon = np.zeros([i_number, 8, rolling_horizon])
    
    for i in range(i_number):
        q_current = calculate_queue_dynamics(i_number, T, step, update_win, arrival, total_queue, start_time_total, end_time_total, link_type, up_inter, up_movement, travel_t, saturation, delta_t, p, rolling_horizon)[0]
        
        q_history_hi1 = np.zeros([i_number, 8, rolling_horizon])
        q_out_signal1 = np.zeros([i_number, 8, rolling_horizon])
        
        for m in range(8):
            if link_type[i, m] == 1:
                for t in range(rolling_horizon):
                    q_in_opt_horizon[i, m, t] = arrival
            else:
                up_inter_id = inters_list.index(up_inter[i, m])
                up_movement_id = up_movement[i, m]
                for t in range(rolling_horizon):
                    if travel_t[i, m] == rolling_horizon:
                        q_history_hi1[int(up_inter_id), int(up_movement_id[0]), :] = q_current[int(up_inter_id), int(up_movement_id[0]), :]
                        q_history_hi1[int(up_inter_id), int(up_movement_id[1]), :] = q_current[int(up_inter_id), int(up_movement_id[1]), :]
                    else:
                        diff = travel_t[i, m] - rolling_horizon
                        if t <= diff:
                            q_history_hi1[int(up_inter_id), int(up_movement_id[0]), t] = total_queue[int(step - diff + t)][int(up_inter_id)][int(up_movement_id[0])]
                            q_history_hi1[int(up_inter_id), int(up_movement_id[1]), t] = total_queue[int(step - diff + t)][int(up_inter_id)][int(up_movement_id[1])]
                        else:
                            q_history_hi1[int(up_inter_id), int(up_movement_id[0]), t] = q_current[int(up_inter_id), int(up_movement_id[0]), int(t - diff)]
                            q_history_hi1[int(up_inter_id), int(up_movement_id[1]), t] = q_current[int(up_inter_id), int(up_movement_id[1]), int(t - diff)]

                    for t in range(rolling_horizon):
                        v_out0 = min(q_history_hi1[int(up_inter_id), int(up_movement_id[0]), t], q_out_signal1[int(up_inter_id), int(up_movement_id[0]), t])
                        v_out1 = min(q_history_hi1[int(up_inter_id), int(up_movement_id[1]), t], q_out_signal1[int(up_inter_id), int(up_movement_id[1]), t])
                        q_in_opt_horizon[i, m, t] = (v_out0 + v_out1) * p
    
    return q_in_opt_horizon


def initialize_fixed_timing(i_number, initial_start, initial_end, start_time_total, end_time_total):
    # Initialize fixed signal plan in the first optimization
    s = np.zeros((i_number, len(initial_start)))
    e = np.zeros((i_number, len(initial_end)))
    for i in range(i_number):
        s[i, :] = initial_start
        e[i, :] = initial_end
    start_time_total.extend([s] * 5)
    end_time_total.extend([e] * 5)
