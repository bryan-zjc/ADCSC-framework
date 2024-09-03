# -*- coding: utf-8 -*-
"""
Created on Sun Aug 23 17:05:39 2024

@author: Jichen Zhu
"""

import numpy as np
import traci
import optparse
from sumolib import checkBinary
import sys

from utils import params
from utils import funs

### Parameters ###
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

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

options = get_options()
# This script has been called from the command line. It will start sumo as a
# server, then connect and run
if options.nogui:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')
traci.start([sumoBinary, "-c", "env/demo.sumocfg",
              "--device.emissions.probability", "1",
              "--tripinfo-output", 
              "Results/tripinfo_test.xml",
              "--vehroute-output",
              "Results/routes_test.xml"])

C = np.zeros([i_number,8])
link_len = np.zeros([i_number,8])
travel_t = np.zeros([i_number,8])

mapping = {
    0: [0, 2],
    1: [1, 3],
    2: [4, 6],
    3: [5, 7]
}

for i in range(len(link)):
    for l in range(len(link[i])):
        l_id = str(link[i][l])
        l_len = traci.lane.getLength(l_id+'_0')
        m_id = mapping[l]
        
        C[i][m_id[0]] = C[i][m_id[1]] = int(l_len/7)
        link_len[i][m_id[0]] = link_len[i][m_id[1]] = l_len
        travel_t[i][m_id[0]] = travel_t[i][m_id[1]] = int(l_len/vf)

# Warm-up time of 500s
rolling_step = 500
step = 0
s = np.zeros([i_number, 8])
e = np.zeros([i_number, 8])
start_time = [[] for _ in range(i_number)]
end_time = [[] for _ in range(i_number)]
green_time = [[] for _ in range(i_number)]
control_m = [[] for _ in range(i_number)]
total_solve_time = []
failure_count = 0 # Count the number of times the model fails to solve
simulation_step = 3600
teleport_list = []

start_time_total = []
end_time_total = []
total_queue = []

opt_start_point = np.zeros(i_number)
opt_end_point = np.zeros(i_number)

q_current = np.zeros([i_number,8,rolling_horizon])

for i in range(i_number):
    exec("inter_" + str(i)+"=[]")

while step < simulation_step:
    traci.simulationStep()
    
    total_queue.append(funs.get_initial_queue())
    
    if step < 500:
        step += 1
        
    else:
        if step == rolling_step: # At this point, do two things: 1. Predict vehicle arrival for the next optimization period and optimize; 2. Execute the previously optimized plan
            if step == 500:
                # 1. Predict vehicle arrival for the next optimization period and optimize
                # Fixed timing plan
                initial_start = [0,0,15,15,30,30,45,45]
                initial_end = [12,12,27,27,42,42,57,57]
                # Assume the previous signal timing plan is fixed timing
                funs.initialize_fixed_timing(i_number, initial_start, initial_end, start_time_total, end_time_total)
                
                # Since it's the first optimization, the previous plan is fixed timing, and the upcoming plan is also assumed to be fixed timing
            q_current, q_start_opt = funs.calculate_queue_dynamics(i_number, T, step, update_win, arrival, total_queue, start_time_total, end_time_total, link_type, up_inter, up_movement, travel_t, saturation, delta_t, p, rolling_horizon)
            q_in_opt_horizon = funs.calculate_opt_horizon(i_number, step, rolling_horizon, arrival, total_queue, start_time_total, end_time_total, link_type, up_inter, up_movement, travel_t, saturation, delta_t, p)

            for i in range(i_number):
                s[i,], e[i,], solve_time = funs.optimization_signal(i,q_in_opt_horizon,q_start_opt,C)
                total_solve_time.append(solve_time)
                exec("inter_" + str(i)+".append(solve_time)")
                    
            start_time_total.append(s)
            end_time_total.append(e)
            # Obtained the optimization results for the next optimization phase
            
            # 2. Execute the previously optimized plan (originally the optimized plan to be executed was -1, but after the above optimization, new plans were added, so the plan to be executed is -2)
            implement_s = start_time_total[-2]
            implement_e = end_time_total[-2]
            for i in range(i_number):
                start_time[i], end_time[i], green_time[i], control_m[i] = funs.get_signal_time(i,implement_s[i],implement_e[i],step)

            phases = [[[] for _ in range(rolling_horizon)] for i in range(i_number)]
            
            phases_index =  phases
            for i in range(i_number):
                for t in range(rolling_horizon):
                    if len(funs.find_all_index(start_time[i], step+t)) != 0:
                        index = funs.find_all_index(start_time[i], step+t)
                        for j in index:
                            m = control_m[i][j]
                            start_transfer = int(start_time[i][j]-step)
                            end_transfer = min(int(end_time[i][j]-step+1),rolling_horizon)
                            for tt in range(start_transfer, end_transfer):
                                phases[i][tt].append(m)
            for i in range(i_number):
                for t in range(rolling_horizon):
                    if phases[i][t] == []:
                        phases_index[i][t] = all_red_phase
                    else:
                        try:
                            phases_index[i][t] = allowed_phase[allowed_movements1.index(phases[i][t])]
                        except:
                            phases_index[i][t] = allowed_phase[allowed_movements2.index(phases[i][t])] 
            # Determine the phase to be executed at the decision point at this moment
            for i in range(i_number):
                t = step%rolling_horizon
                inter_id = inters_list[i]
                traci.trafficlight.setRedYellowGreenState(str(inter_id),phases_index[i][t])
            
            rolling_step += rolling_horizon
            step += 1
                
        else: # At this point, it is not a decision point, need to determine the signal timing plan to execute
            # Determine the phase to be executed at the decision point at this moment
            for i in range(i_number):
                t = step%rolling_horizon
                inter_id = inters_list[i]
                traci.trafficlight.setRedYellowGreenState(str(inter_id),phases_index[i][t])
            step += 1
                
traci.close()       
sys.stdout.flush()

for ii in range(i_number):
    exec("inter_%i_df = pd.DataFrame(columns=['Solution Time (s)', 'Intersection Index', 'Control Method'])"%ii)
    exec("inter_%i_df['Solution Time (s)']=inter_%i"%(ii,ii))
    exec("inter_%i_df['Intersection Index']=%i"%(ii,ii+1))
    exec("inter_%i_df['Control Method']='Asynchronous'"%(ii))
    exec("inter_%i_df.to_csv('solution_time/inter_%i.csv',index = False)"%(ii,ii))


