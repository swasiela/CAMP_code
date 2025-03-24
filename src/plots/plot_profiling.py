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
import re
import matplotlib.pyplot as plt

# Step 1: Specify the folder path containing files to read
folder_path = '/home/swasiela/CAMP/src/results/Profiling/Quad/RRTstar/'  # Replace with your folder path

# Set chart type: "bar" or "pie"
chart_type = "pie"  # Change to "bar" for bar chart

# Step 2: Initialize a dictionary to hold percentages for each file
all_files_percentages = {}

# Step 3: Define the desired file order (assuming filenames contain these substrings)
desired_order = ["RRT*", "SARRT*", "SST*", "SASST*"]

# Step 4: Iterate through each file in the folder
for filename in os.listdir(folder_path):
    file_path = os.path.join(folder_path, filename)
    
    if filename.endswith('.txt'):
        with open(file_path, 'r') as file:
            data = file.read()

        # Step 5: Parse the data in each file
        pattern = r"ID:(\w+);Running time:([\deE\+\-\.]+)"
        matches = re.findall(pattern, data)

        # Step 6: Calculate total times for each ID in the file
        times_by_id = {}
        for id_name, running_time in matches:
            running_time = float(running_time)
            times_by_id[id_name] = times_by_id.get(id_name, 0) + running_time

        # Rename algorithms for consistency
        renamed_times = {}
        for key, value in times_by_id.items():
            if key == "ode":
                key = "ODEs"
            renamed_times[key] = value
            
        # Calculate total time and percentage contribution for each ID
        total_time = sum(renamed_times.values())
        percentages = {id_name: (time / total_time) * 100 for id_name, time in renamed_times.items()}

        # Store the percentages with the filename as key (without .txt)
        all_files_percentages[os.path.splitext(filename)[0]] = percentages

# Step 7: Sort files based on desired order
sorted_files = []
seen_files = set()

for key in desired_order:
    matching_files = [f for f in all_files_percentages if key == f]  # Remove ".txt" from comparison
    for file in matching_files:
        if file not in seen_files:
            sorted_files.append(file)
            seen_files.add(file)

# Define consistent colors for each algorithm
colors = plt.cm.tab20.colors
algorithm_colors = {
    "CC": colors[0], "Cost": colors[1], "NN": colors[2], "Sampling": colors[3], 
    "Local Plan": colors[4], "Tree": colors[5], "ODEs": colors[6], "GRU": colors[7]
}

if chart_type == "bar":
    # Step 8: Plot stacked bar chart
    plt.figure(figsize=(12, 8))

    # Algorithms to display (keys from file data)
    algorithms = sorted({id_name for file_data in all_files_percentages.values() for id_name in file_data})

    # Ensure "GRU" is processed last
    if "GRU" in algorithms:
        algorithms.remove("GRU")
        algorithms.append("GRU")
        
    bottoms = [0] * len(sorted_files)  # Keeps track of bottom position for stacking

    for idx, algorithm in enumerate(algorithms):
        heights = [all_files_percentages[file].get(algorithm, 0) for file in sorted_files]
        if algorithm == "ode":
            algorithm = "ODEs"
        plt.bar(sorted_files, heights, bottom=bottoms, label=algorithm, color=algorithm_colors.get(algorithm))
        
        # Add percentage labels to each segment in the bars
        for i, height in enumerate(heights):
            if height > 3:  # Only add text if there is a non-zero percentage
                plt.text(i, bottoms[i] + height / 2, f"{height:.1f}%", ha='center', va='center', fontsize=25, color="black")
        
        # Update bottoms for stacking
        bottoms = [bottoms[i] + heights[i] for i in range(len(sorted_files))]

    # Add labels 
    # Prepare x-axis labels without ".txt"
    labels = [os.path.splitext(f)[0] for f in sorted_files]  # Remove ".txt"
    plt.ylabel('Contribution to Total Time (%)', fontsize=35)
    plt.legend(title="Procedures", bbox_to_anchor=(1.05, 1), loc='upper left', title_fontsize=35 ,fontsize=35)

    plt.yticks(fontsize=25)
    plt.xticks(range(len(sorted_files)), labels, rotation=45, ha='right', fontsize=35)
    plt.tight_layout()
    plt.show()

elif chart_type == "pie":
    # Step 9: Plot pie charts with consistent colors
    for filename in sorted_files:
        percentages = all_files_percentages[filename]
        labels = [k for k, v in percentages.items() if v > 1]  # Ignore tiny values
        sizes = [v for k, v in percentages.items() if v > 1]
        colors_pie = [algorithm_colors.get(k, colors[len(algorithm_colors) % 20]) for k in labels]  # Assign colors

        plt.figure(figsize=(8, 8))
        plt.pie(sizes, labels=labels, autopct='%1.1f%%', colors=colors_pie, startangle=140, textprops={'fontsize': 35})
        plt.title(f"{filename}", fontsize=45)  # No ".txt"
        plt.axis('equal')  
        plt.show()