# Software License Agreement (BSD License)
# 
# Copyright (c) 2024, LAAS-CNRS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Simon WASIELA 

import os
from os import listdir
from os.path import isfile, join
from scipy.interpolate import interp1d
import scipy.optimize

import subprocess
import sys
import math
import numpy as np
import pandas as pd
import seaborn as sns
from stl import mesh
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits import mplot3d
from math import *

from matplotlib.collections import PatchCollection
from matplotlib import patches  # Import patches for rectangles in PatchCollection

PERCENTAGE = True

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

# ------------------------------------ Plotters section ------------------------------------
def is_dark_color(color):
    """ Check if a color is dark based on its RGB values. """
    r, g, b, *_ = mcolors.to_rgba(color)  # Convert color to RGBA
    brightness = 0.299 * r + 0.587 * g + 0.114 * b
    return brightness < 0.4  # Adjust this threshold as needed

def color_distance(c1, c2):
    """ Calculate Euclidean distance between two colors in RGB space. """
    return np.sqrt(np.sum((np.array(mcolors.to_rgb(c1)) - np.array(mcolors.to_rgb(c2))) ** 2))

def get_distinct_colors(base_colors, num_colors, min_distance):
    """ Get a specified number of distinct colors from a list of base colors. """
    chosen_colors = []
    for color in base_colors:
        if len(chosen_colors) >= num_colors:
            break
        if all(color_distance(color, c) >= min_distance for c in chosen_colors):
            chosen_colors.append(color)
    return chosen_colors

def apply_filter(state_key, filters):
    """Check if the state_key matches the filter criteria."""
    state_parts = state_key.split('_')
    
    # Case 1: Methods without subfolders (only 'method' in the key)
    if len(state_parts) == 1:
        method = state_parts[0]

        # Match the method filter if present
        if 'method_filter' in filters and filters['method_filter'] and filters['method_filter'] != method:
            return False
    
    # Case 2: Methods with subfolders (key in the form of 'method_radius_nstates')
    elif len(state_parts) >= 5:
        method = state_parts[0]  # The "method" part
        sampling = state_parts[1]  # The "sampling" part
        if len(state_parts) == 5:
            radius = state_parts[2] + "_" + state_parts[3] # The "radius" part
            nstates = state_parts[4]  # The "nstates" part (e.g., "5states")
        if len(state_parts) == 6:
            radius = state_parts[2] + "_" + state_parts[3] + "_" + state_parts[4]  # The "radius" part
            nstates = state_parts[5]  # The "nstates" part (e.g., "5states")

        # Match the filters
        if 'method_filter' in filters and filters['method_filter'] and filters['method_filter'] != method:
            return False
        if 'sampling_filter' in filters and filters['sampling_filter'] and filters['sampling_filter'] != sampling:
            return False
        if 'radius_filter' in filters and filters['radius_filter'] and filters['radius_filter'] != radius:
            return False
        if 'nstates_filter' in filters and filters['nstates_filter'] and filters['nstates_filter'] != nstates:
            return False

    return True

def curves_mean_std(costs, times, color,label, lim, step, ax, patchAlpha = 0.1,marker_rep = 10, marker_line = "x", marker_scale = 7):
    stdY = []
    Q1 = []
    Q3 = []
    arrY = []
    arrX = []
    arrY_max = []
    arrY_min = []
    
    # According to the given time steps construct the mean and std vector until the time limite is reached
    for i in range(0, lim, step):
        yData = []
        
        # Update values for the current time step
        for l in range(len(costs)):
            # Find the current time index where times[l] < i
            current_time_idx = next((index for index, value in reversed(list(enumerate(times[l]))) if value < i), 0)
            yData.append(costs[l][current_time_idx])

        srs_yData = pd.Series(yData)
        arrY.append(srs_yData.mean())
        stdY.append(srs_yData.std())
        Q1.append(np.percentile(srs_yData,25))
        Q3.append(np.percentile(srs_yData,75))
        arrY_max.append(min(srs_yData.max(),Q3[-1]+1.5*(Q3[-1]-Q1[-1])))
        arrY_min.append(max(srs_yData.min(),Q3[-1]-1.5*(Q3[-1]-Q1[-1])))
        arrX.append(i)
       
    arrY_min = pd.Series(arrY_min)
    arrY_max = pd.Series(arrY_max)
    
    Q3 = pd.Series(Q3)
    Q1 = pd.Series(Q1)
    arrY_max = pd.Series(arrY_max)
    arrY_min = pd.Series(arrY_min)
    
    arrY = pd.Series(arrY)
    
    errorboxes = [plt.Rectangle((x , y - (ye/2)), step, ye) for x, y, ye in zip(arrX, arrY, stdY)]
    
    # Create patch collection with specified colour/alpha
    pc = PatchCollection(errorboxes[:-1], facecolor=color, alpha=patchAlpha, edgecolor=color, linestyle='--')
    ax.add_collection(pc)
    
    ax.plot(arrX, arrY, color = color, label ="{}".format(label), drawstyle = "steps-post", markevery=marker_rep, ms= marker_scale, markerfacecolor = "none", lw = 3)

    return Q1,Q3

def plot_filtered_mean_std(state_matrices, filters_list):
    fig, ax = plt.subplots()  # Create figure and axis for plotting
    fig1, ax1 = plt.subplots()  # Create figure and axis for boxplot

    # Get a list of distinct CSS4 colors and filter for dark colors
    css_colors = list(mcolors.CSS4_COLORS.values())  # Get a list of CSS4 colors
    dark_colors = [color for color in css_colors if is_dark_color(color)]  # Filter dark colors
    # Get distinct dark colors according to the number of states
    num_states = len(state_matrices)
    selected_colors = get_distinct_colors(dark_colors, num_states, min_distance=0.15)

    used_colors = set()  # To keep track of used colors
    colors = []  # Initialize an empty list to store colors for curves and boxplots
    
    marker_styles = ['o', 'x', 's', '^', 'D']  # Different markers for variety
    
    labels = []  # To store labels for x-ticks
    planner_costs = []  # To store costs for the box plot
    
    # Find the maximum time value
    lim = 300
    ## Iterate over each state in state_matrices
    for idx, (state_key, matrices) in enumerate(state_matrices.items()):
        # Iterate over each filter combination in filters_list
        for filters in filters_list:
            # Apply the filter combination to the current state
            if apply_filter(state_key, filters):
                # Recorver the optimized costs for box plot
                opt_cost = []
                for l in range(len(matrices['times'])):
                    lim_time_idx = next((index for index, value in reversed(list(enumerate(matrices['times'][l]))) if value < lim), 0)
                    if PERCENTAGE:
                        opt_cost.append(matrices['percentage'][l][lim_time_idx])  # Append the best costs to the planner_costs list
                    else:
                        opt_cost.append(matrices['costs'][l][lim_time_idx])  # Append the best costs to the planner_costs list

                planner_costs.append(opt_cost)  # Append the best costs to the planner_costs list
                labels.append(state_key)  # Append the state_key as label
                
                # Find a new color for the current curve
                for color in selected_colors:
                    if color not in used_colors:  # Check if the color has already been used
                        used_colors.add(color)  # Mark this color as used
                        colors.append(color)  # Add it to the colors list
                        break  # Exit the loop once a new color is found

                # Call the mean_std curve function for this filtered state
                if PERCENTAGE:
                    mat_cost = matrices['percentage']
                else:
                    mat_cost = matrices['costs']
                curves_mean_std(
                    costs=mat_cost,  # Column name in costs_matrix that represents time
                    times=matrices['times'],  # Column name in times_matrix that represents cost
                    color=colors[-1],  # Choose a color from the colormap
                    label=state_key,  # Use the method, radius, and nstates as the label
                    step=2,  # Number of divisions for the x-axis
                    lim=lim,
                    ax=ax,
                    marker_rep=10,  # Example value for marker repetition
                    marker_line=marker_styles[idx % len(marker_styles)],  # Cycle through marker styles
                    patchAlpha=0.1  # Transparency for the error envelope
                )
        
    # Create the box plot
    bplot = ax1.boxplot(planner_costs, 
                       patch_artist=True,  # fill with color
                       labels=labels)  # will be used to label x-ticks

    # Assign colors to each boxplot based on the same colors used for the curves
    for patch, color in zip(bplot['boxes'], colors):  # Use the same colors
        patch.set_facecolor(color)
        
    # Customize plot as needed
    # ax.set_xscale('log')  # Set the x-axis to a logarithmic scale
    # ax.set_yscale('log')  # Set the y-axis to a logarithmic scale
    ax.set_xlabel('Planning time (s)', fontsize = 35)
    ax.set_ylabel('Cost improvement (%)', fontsize = 35)
    ax.tick_params(axis='both', which='major', labelsize=35) 
    ax.tick_params(axis='both', which='minor', labelsize=35)
    ax.legend(loc='center right',fontsize=40)
    ax.grid(True, which='both', axis='y', linestyle='--', color='gray', alpha=0.7)
    ax.grid(True, which='both', axis='x', linestyle='--', color='gray', alpha=0.7)
    plt.show()
# --------------------------------------------------------------------------------------

# ------------------------------------ Parsing section ------------------------------------
def extract_data_from_file(file_path):
    costs = []
    times = []
    percentage_improvements = []

    # Open and read the file
    with open(file_path, 'r') as f:
        lines = f.readlines()
        
        # Initialize flags to track which data we are currently reading
        readingTraj = False
        readingCost = False
        readingTime = False

        # Process each line
        for line in lines:
            if "Trajectory" in line:
                readingTraj = True
                readingCost = False
                readingTime = False
                continue
            if "Cost" in line:
                readingTraj = False
                readingCost = True
                readingTime = False
                continue
            if "Time" in line:
                readingTraj = False
                readingCost = False
                readingTime = True
                continue

            # Extract cost and time data based on the current context
            if readingCost:
                try:
                    # Convert cost to float and append to the costs list
                    cost_value = float(line.strip())
                    costs.append(cost_value)
                except ValueError:
                    pass  # Skip lines that don't contain valid numbers
            elif readingTime:
                try:
                    # Convert time to float and append to the times list
                    time_value = float(line.strip())
                    times.append(time_value)
                except ValueError:
                    pass  # Skip lines that don't contain valid numbers

    # Calculate the percentage improvement for each cost relative to the initial cost
    if costs:
        initial_cost = costs[0]
        for cost in costs:
            percentage_improvement = (cost / initial_cost) * 100
            percentage_improvements.append(percentage_improvement)
            
    return costs, percentage_improvements, times

def process_folder(folder_path):
    all_costs = []
    all_times = []
    all_percentage_improvements = []

    # Walk through the folder and process .txt files only
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".txt"):
                # Get the full path of the file
                file_path = os.path.join(root, file)
                
                # Extract costs and times from this file
                costs, improves, times = extract_data_from_file(file_path)
                
                # Append the data from this file to the folder-level lists
                all_costs.append(costs)
                all_times.append(times)
                all_percentage_improvements.append(improves)

    return all_costs, all_percentage_improvements, all_times

def process_directory_by_state(base_directory):
    state_matrices = {}

    # Traverse the base directory ('method') to find 'radius' and 'n states' folders
    for root, dirs, files in os.walk(base_directory):
        path_parts = root.split(os.sep)  # Split the root path to find 'method', 'radius', and 'n states'

        # Check if we are in a method folder (e.g., Shortcut or Stomp)
        method_folder = path_parts[-1]

        # Case 1: Handling folders with subfolders ('radius' and 'n states'), e.g., ExtendedShortcut
        if len(path_parts) >= 3 and path_parts[-1].endswith('states') and path_parts[-1].split('states')[0].isdigit():
            # Get the required folder names
            method_folder = path_parts[-4]  # 'method' folder (4rd from last)
            sampling_folder = path_parts[-3]  # 'sampling' folder (3rd from last)
            radius_folder = path_parts[-2]  # 'radius' folder (2nd from last)
            states_folder = path_parts[-1]  # 'n states' folder (last)

            # Combine them to create the key
            state_key = f"{method_folder}_{sampling_folder}_{radius_folder}_{states_folder}"

        # Case 2: Handling methods without subfolders (e.g., Stomp or Shortcut)
        elif len(path_parts) >= 2 and not dirs:  # No subdirectories in the method folder
            state_key = f"{method_folder}"

        else:
            continue  # Skip irrelevant folders

        # Process this folder to get cost and time matrices
        costs_matrix, percentage_matrix, times_matrix = process_folder(root)

        # Store the matrices using the combined key
        state_matrices[state_key] = {
            'costs': costs_matrix,
            'percentage': percentage_matrix,
            'times': times_matrix
        }

    return state_matrices
# --------------------------------------------------------------------------------------

# ------------------------------------ Main section ------------------------------------
if __name__ == '__main__':
    # relative_folder = os.path.join('src', 'results', 'LocalOpt', 'Empty', 'Methods')
    # relative_folder = os.path.join('src', 'results', 'LocalOpt', 'Window_pilar', 'Methods')
    relative_folder = os.path.join('src', 'results', 'LocalOpt', 'Corridor', 'Methods')

    # Process the directory to get a dictionary of matrices per subfolder
    state_matrices = process_directory_by_state(relative_folder)

    # Plot curves according to specified keys
    # filters_list = [{"method_filter": "ExtendedShortcut", "sampling_filter": "UniformSampling", "radius_filter": "radius_0_01", "nstates_filter": None}]
    
    # filters_list = [
    #     {"method_filter": "ExtendedShortcut", "sampling_filter": "GaussianSampling", "radius_filter": "radius_0_1", "nstates_filter": "1states"},
    #     {"method_filter": "ExtendedShortcut", "sampling_filter": "GaussianSampling", "radius_filter": "radius_0_01", "nstates_filter": "1states"},
    #     {"method_filter": "ExtendedShortcut", "sampling_filter": "UniformSampling", "radius_filter": "radius_0_1", "nstates_filter": "1states"},
    #     {"method_filter": "ExtendedShortcut", "sampling_filter": "UniformSampling", "radius_filter": "radius_0_01", "nstates_filter": "1states"},]
    
    filters_list = [
        {"method_filter": "Shortcut", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},
        {"method_filter": "COBYLA", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},
        {"method_filter": "STOMP", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},]
    
    # filters_list = [
    #     {"method_filter": "ExtendedShortcut01", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},
    #     {"method_filter": "ExtendedShortcut001", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},
    #     {"method_filter": "Shortcut", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},
    #     {"method_filter": "Nlopt", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},
    #     {"method_filter": "Stomp", "sampling_filter": None, "radius_filter": None, "nstates_filter": None},]

    plot_filtered_mean_std(state_matrices, filters_list)
    
    print("Done")
# --------------------------------------------------------------------------------------