import sys, os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib import collections  as mc

def gen_ip_table(ip_file_name, total_frames):
    with open(ip_file_name) as f:
        table = np.zeros((total_frames, total_frames), dtype=float)
        lines = f.readlines()
        for line in lines:
            ip_pair = line.split()
            f1 = int(ip_pair[0])
            f2 = int(ip_pair[1])
            ip = float(ip_pair[2])
            table[f1][f2] = ip
            table[f2][f1] = ip
    return table

def gen_path(traj_file_name, total_frames):
    table = np.zeros(( total_frames, 2), dtype=float)
    line_num = 0
    with open(traj_file_name) as f:
        lines = f.readlines()
        for line in lines:
            if (line_num == total_frames): break
            ip_pair = line.split()
            
            table[line_num][0] = ip_pair[3]
            table[line_num][1] = ip_pair[11]            
            line_num += 1

    return table
    

def plot_ip_table_single_frame(ip_table, traj_table, frame_id):
    fig = plt.figure()
    traj_shape = traj_table.shape
    lines = []
    for i in range(traj_shape[0]-1):
        lines.append([(traj_table[i,0],traj_table[i,1]),(traj_table[i+1,0],traj_table[i+1,1]) ])
    print(len(lines))
    lc = mc.LineCollection(lines, linewidths=1)

    ax1 = fig.add_subplot(1, 1, 1)

    lcs = []
    for j in range(traj_table.shape[0]):
        if ip_table[frame_id][j] > 0.9:
            matched_id = j
            lcs.append([(traj_table[frame_id][0], traj_table[frame_id][1]),(traj_table[j][0],traj_table[j][1])])
            #print(lcs[len(lcs)-1])
            #print(frame_id, j, ip_table[frame_id][j])
    lcs_collection = mc.LineCollection(lcs, linewidths=1, colors='g')
    
    ax1.add_collection(lc)
    ax1.add_collection(lcs_collection)
    ax1.autoscale()
    #fig, ax = plt.subplots()
    #ax.add_collection(lc)
    #ax.autoscale()
    #ax.margins(0.1)
    plt.show()

if __name__ == "__main__":
    ip_file_name = sys.argv[1]
    traj_file_name = sys.argv[2]
    total_frames = int(sys.argv[3])
    frame_id = int( sys.argv[4])


    ip_table = gen_ip_table(ip_file_name, total_frames)
    traj_table = gen_path(traj_file_name, total_frames)
    plot_ip_table_single_frame(ip_table,traj_table, frame_id)
    
