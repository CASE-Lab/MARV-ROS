#!/usr/bin/env python3

import waypoint_algorithm as wpa
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

waypoints = np.array([  [70, -30, 10, 5],
                        [71, 30, 10, 5],
                        [-100, -5, 10, 5],
                        [-200, 20, 10 ,5]])
parameters = np.array([2, 1, 1.5, 20])
df_matlab = pd.read_csv('C:/Users/lgnds/Documents/GitHub/WaveRunner-Controller/colcon_ws/src/waverunner_scenarios/library/wp_gen.csv')

wp_gen = wpa.waypoint_algorithm(waypoints,parameters)

vel_ref_out = []
head_ref_out = []

for x,y,psi in zip(df_matlab.eta_x, df_matlab.eta_y, df_matlab.eta_psi):
    wp_gen.update(np.array([x,y,psi]))
    vel_ref_out.append(wp_gen.vel_ref)
    head_ref_out.append(wp_gen.head_ref)

print('Finished')

df_ver = df_matlab[['head_ref', 'vel_ref']]

df_out = pd.DataFrame(list(zip(head_ref_out,vel_ref_out)),columns=['head_ref_python','vel_ref_python'])

#df_ver['head_ref_python'] = head_ref_out
#df_ver['vel_ref_python'] = vel_ref_out
print(df_out)
print(df_ver)

print('Plot')
plt.close("all")
plt.figure()
df_out.plot()