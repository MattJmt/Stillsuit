import plotly.graph_objs as go
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
import pandas as pd

filename = r"C:\Users\matja\Documents\_GitHub\Master-Thesis\stillsuit_rpi\DYNAMICS_experiments\Chiara\Chiara_Steel_Penguin_Bowden_slow_1.csv"
data = pd.read_csv(filename)

t = data["t"]
p_cur_m1 = data["p_cur_m1"]
p_des_m1 = data["p_des_m1"]
v_des_m1 = data["v_des_m1"]
v_cur_m1 = data["v_cur_m1"]
t_des_m1 = data["t_des_m1"]
t_cur_m1 = data["t_cur_m1"]
gyr_z_1 = data["gzfilt_S0"]
pitch_1 = data["pitch_S0"]
stance_1 = data["stance_S0"]
HS_1 = data["HS_S0"]
TO_1 = data["TO_S0"]

pretension_1 = data["pretension_m1"]

p_cur_m2 = data["p_cur_m2"]
p_des_m2 = data["p_des_m2"]
v_des_m2 = data["v_des_m2"]
v_cur_m2 = data["v_cur_m2"]
t_des_m2 = data["t_des_m2"]
t_cur_m2 = data["t_cur_m2"]
gyr_z_2 = data["gzfilt_S1"]
pitch_2 = data["pitch_S1"]
stance_2 = data["stance_S1"]
pretension_2 = data["pretension_m2"]
HS_2 = data["HS_S1"]
TO_2 = data["TO_S1"]

load_cell_force = data["force_S1"]

"""
Position plot
"""

# fig_position = go.Figure()
# fig_position.add_trace(go.Scatter(x=t, y=p_cur_m1, mode='lines', name='p_cur_m1', line=dict(color='red', width=2)))
# fig_position.add_trace(go.Scatter(x=t, y=p_cur_m2, mode='lines', name='p_cur_m2', line=dict(color='blue', width=2)))
# fig_position.update_layout(title='Position', xaxis_title='time (s)', yaxis_title='Position (rad)', showlegend=True)
# fig_position.show()

""" 
Velocity Plots
"""

# sos = signal.butter(3, 20, 'lp', fs=200, output='sos')
# so2 = signal.butter(3, 50, 'lp', fs=200, output='sos')

# V1_butter_ref = signal.sosfiltfilt(sos, v_cur_m1)
# V2_butter_ref = signal.sosfiltfilt(sos, v_cur_m2)

# fig_velocity = go.Figure()
# fig_velocity.add_trace(go.Scatter(x=t, y=v_cur_m1, mode='lines', name='v_cur_m1', line=dict(color='red', width=2, dash='dash')))
# fig_velocity.add_trace(go.Scatter(x=t, y=v_cur_m2, mode='lines', name='v_cur_m2', line=dict(color='blue', width=2, dash='dash')))
# fig_velocity.add_trace(go.Scatter(x=t, y=V1_butter_ref, mode='lines', name='V1_butter_ref', line=dict(color='green', width=2)))
# fig_velocity.add_trace(go.Scatter(x=t, y=V2_butter_ref, mode='lines', name='V2_butter_ref', line=dict(color='magenta', width=2)))
# fig_velocity.add_trace(go.Scatter(x=t, y=v_des_m1, mode='lines', name='v_des_m1', line=dict(color='purple', width=2)))
# fig_velocity.add_trace(go.Scatter(x=t, y=v_des_m2, mode='lines', name='v_des_m2', line=dict(color='orange', width=2)))
# fig_velocity.update_layout(title='Velocity', xaxis_title='time (s)', yaxis_title='Velocity (rad/s)', showlegend=True)
# fig_velocity.show()


""" 
Torque Plot 
"""

# Torque plot
sos = signal.butter(3, 10, 'lp', fs=200, output='sos')
T1_butter_ref = signal.sosfiltfilt(sos, t_cur_m1)
T2_butter_ref = signal.sosfiltfilt(sos, t_cur_m2)
fig_torque = go.Figure()
fig_torque.add_trace(go.Scatter(x=t, y=t_cur_m1, mode='lines', name='t_cur_m1', line=dict(color='red', width=2, dash='dot')))
fig_torque.add_trace(go.Scatter(x=t, y=t_cur_m2, mode='lines', name='t_cur_m2', line=dict(color='blue', width=2, dash='dot')))
fig_torque.add_trace(go.Scatter(x=t, y=T1_butter_ref, mode='lines', name='T1_butter_ref', line=dict(color='green', width=2)))
fig_torque.add_trace(go.Scatter(x=t, y=T2_butter_ref, mode='lines', name='T2_butter_ref', line=dict(color='cyan', width=2)))
fig_torque.add_trace(go.Scatter(x=t, y=t_des_m1, mode='lines', name='t_des_m1', line=dict(color='brown', width=2)))
fig_torque.add_trace(go.Scatter(x=t, y=t_des_m2, mode='lines', name='t_des_m2', line=dict(color='black', width=2)))
# fig_torque.add_trace(go.Scatter(x=t, y=pretension_1, mode='lines', name='pretension1', line=dict(color='darkviolet', width=2)))
# fig_torque.add_trace(go.Scatter(x=t, y=pretension_2, mode='lines', name='pretension2', line=dict(color='fuchsia', width=2)))
fig_torque.update_layout(title='Torque', xaxis_title='time (s)', yaxis_title='Torque (Nm)', showlegend=True)
fig_torque.show()

"""
IMU Plots
"""
fig_imu = go.Figure()
fig_imu.add_trace(go.Scatter(x=t, y=gyr_z_1, mode='lines', name='gyr_z_1', line=dict(color='red', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=-gyr_z_2, mode='lines', name='gyr_z_2', line=dict(color='blue', width=2)))
# fig_imu.add_trace(go.Scatter(x=t1, y=heading_1, mode='lines', name='heading_1', line=dict(color='red', width=2)))
# fig_imu.add_trace(go.Scatter(x=t1, y=heading_2, mode='lines', name='heading_2', line=dict(color='blue', width=2)))
# fig_imu.add_trace(go.Scatter(x=t, y=pitch_1, mode='lines', name='pitch_1', line=dict(color='orange', width=2)))
# fig_imu.add_trace(go.Scatter(x=t, y=pitch_2, mode='lines', name='pitch_2', line=dict(color='blueviolet', width=2)))
# fig_imu.add_trace(go.Scatter(x=t1, y=roll_1, mode='lines', name='roll_1', line=dict(color='red', width=2)))
# fig_imu.add_trace(go.Scatter(x=t1, y=roll_2, mode='lines', name='roll_2', line=dict(color='blue', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=stance_1, mode='lines', name='stance_1', line=dict(color='pink', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=stance_2, mode='lines', name='stance_2', line=dict(color='yellow', width=2)))
# fig_imu.add_trace(go.Scatter(x=t, y=T1_butter_ref, mode='lines', name='T1_butter_ref', line=dict(color='green', width=2)))
# fig_imu.add_trace(go.Scatter(x=t, y=T2_butter_ref, mode='lines', name='T2_butter_ref', line=dict(color='pink', width=2)))
# fig_imu.add_trace(go.Scatter(x=t, y=t_des_m1, mode='lines', name='t_des_m1', line=dict(color='brown', width=2)))
# fig_imu.add_trace(go.Scatter(x=t, y=t_des_m2, mode='lines', name='t_des_m2', line=dict(color='black', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=HS_1, mode='lines', name='HS_S1', line=dict(color='orange', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=TO_1, mode='lines', name='TO_S1', line=dict(color='purple', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=HS_2, mode='lines', name='HS_S2', line=dict(color='cyan', width=2)))
fig_imu.add_trace(go.Scatter(x=t, y=TO_1, mode='lines', name='TO_S2', line=dict(color='red', width=2)))
fig_imu.update_layout(title='IMU', xaxis_title='time (s)', yaxis_title='Position (rad)', showlegend=True)
fig_imu.show()

"""
Load Cell Plot
"""
sos = signal.butter(3, 10, 'lp', fs=200, output='sos')
load_cell_filt = signal.sosfiltfilt(sos, load_cell_force)
fig_position = go.Figure()
fig_position.add_trace(go.Scatter(x=t, y=load_cell_force, mode='lines', name='loadcell_force', line=dict(color='red', width=2)))
fig_position.add_trace(go.Scatter(x=t, y=load_cell_filt, mode='lines', name='load_cell_filt', line=dict(color='blue', width=2)))
fig_position.update_layout(title='Load Cell Force', xaxis_title='time (s)', yaxis_title='Force (N)', showlegend=True)
fig_position.show()

"""
IMU PLOTS Presentation
"""
sos = signal.butter(3, 10, 'lp', fs=200, output='sos')
imu_S1 = signal.sosfiltfilt(sos, gyr_z_1)
imu_S2 = signal.sosfiltfilt(sos, gyr_z_2)
# imu_S1 = signal.sosfiltfilt(sos, t_des_m1)
# imu_S2 = signal.sosfiltfilt(sos, t_des_m2)

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Create the plot
plt.figure(figsize=(12, 8))
sns.lineplot(x=t, y=imu_S1, color = "springgreen", label = "IMU R", linewidth = 3)
sns.lineplot(x=t, y=-imu_S2, color = "darkgreen", label = "IMU L", linewidth = 3)
# sns.lineplot(x=t, y=imu_S1, color = "deepskyblue", label = "Motor R", linewidth = 3)
# sns.lineplot(x=t, y=-imu_S2, color = "steelblue", label = "Motor L", linewidth = 3)
sns.lineplot(x=t, y=0, color = "black", linewidth = 2, linestyle = "--")


for i in range(len(t)-1):
    if stance_1[i] == 1:
        plt.axvspan(t[i], t[i+1], facecolor='grey', alpha=0.15)

# Customize the plot
plt.xlabel('Time (s)', fontsize=50)
# plt.ylabel('Torque (Nm)', fontsize=50)
plt.ylabel('Ang. Vel. (deg/s)', fontsize=50)
plt.tick_params(axis='both', which='major', labelsize=40)
plt.tick_params(axis='both', which='minor', labelsize=40)
plt.xlim(45,48)
plt.legend(fontsize = 40, loc='center left', bbox_to_anchor=(1, 0.5))
plt.title('')
# plt.ylim(-0.1,3.2)
plt.tight_layout()
# plt.savefig('Dynamic_tdes_wstance_longer.png', format='png')
# plt.savefig('Dynamic_tdes_wstance_longer.svg', format='svg')
plt.show()
