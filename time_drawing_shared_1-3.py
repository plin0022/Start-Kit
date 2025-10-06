import json
import os
import matplotlib.pyplot as plt
from collections import defaultdict

# map name

# map_name = 'sortation_small'

# map_name = 'room-64-64-8'

map_name = 'ost003d'

# map_name = 'warehouse_large'


# Path to the directories containing the output files for each algorithm

output_dir_algorithm_1 = f'./{map_name}/traffic_mdd_float_testing'
output_dir_algorithm_2 = f'./{map_name}/traffic_mdd_float_shared_testing'
output_dir_algorithm_3 = f'./{map_name}/traffic_paper_testing'
output_dir_algorithm_4 = f'./{map_name}/pibt_testing'
output_dir_algorithm_5 = f'./{map_name}/traffic_mdd_float_shared_testing_fw'
output_dir_algorithm_6 = f'./{map_name}/traffic_mdd_float_shared_testing_fw_init'
output_dir_algorithm_7 = f'./{map_name}/traffic_paper_testing_fw'



# Function to extract and average response time by team size
def extract_and_average_response_time(output_dir):
    team_size_to_times = defaultdict(list)

    for output_file in os.listdir(output_dir):
        if output_file.endswith('_output.json'):
            output_file_path = os.path.join(output_dir, output_file)

            with open(output_file_path, 'r') as file:
                data = json.load(file)

            team_size = data.get('teamSize')
            planner_times = data.get('plannerTimes', [])

            if team_size is not None and planner_times:
                avg_planner_time = sum(planner_times) / len(planner_times)
                team_size_to_times[team_size].append(avg_planner_time)

    # Compute the average response time for each team size
    avg_time_results = {team_size: sum(times) / len(times) for team_size, times in team_size_to_times.items()}
    return sorted(avg_time_results.items())  # Return sorted by teamSize


# Get averaged response time data for each algorithm
avg_time_1 = extract_and_average_response_time(output_dir_algorithm_1)
avg_time_2 = extract_and_average_response_time(output_dir_algorithm_2)
avg_time_3 = extract_and_average_response_time(output_dir_algorithm_3)
avg_time_4 = extract_and_average_response_time(output_dir_algorithm_4)
avg_time_5 = extract_and_average_response_time(output_dir_algorithm_5)
avg_time_6 = extract_and_average_response_time(output_dir_algorithm_6)
avg_time_7 = extract_and_average_response_time(output_dir_algorithm_7)



# Extract x (team sizes) and y (average response time) for plotting
team_sizes_time_1, avg_response_time_1 = zip(*avg_time_1) if avg_time_1 else ([], [])
team_sizes_time_2, avg_response_time_2 = zip(*avg_time_2) if avg_time_2 else ([], [])
team_sizes_time_3, avg_response_time_3 = zip(*avg_time_3) if avg_time_3 else ([], [])
team_sizes_time_4, avg_response_time_4 = zip(*avg_time_4) if avg_time_4 else ([], [])
team_sizes_time_5, avg_response_time_5 = zip(*avg_time_5) if avg_time_5 else ([], [])
team_sizes_time_6, avg_response_time_6 = zip(*avg_time_6) if avg_time_6 else ([], [])
team_sizes_time_7, avg_response_time_7 = zip(*avg_time_7) if avg_time_7 else ([], [])



# Plot the relationship between team size and average response time
plt.figure(figsize=(10, 6))

plt.plot(team_sizes_time_1, avg_response_time_1, color='b', marker='o', linestyle='-', label='PTFO')
plt.plot(team_sizes_time_2, avg_response_time_2, color='r', marker='x', linestyle='-', label='PTFO_S')
plt.plot(team_sizes_time_3, avg_response_time_3, color='g', marker='^', linestyle='-', label='TFO')
plt.plot(team_sizes_time_4, avg_response_time_4, color='purple', marker='s', linestyle='-', label='PIBT')
plt.plot(team_sizes_time_5, avg_response_time_5, color='orange', marker='x', linestyle='-', label='PTFO_S_Re')
plt.plot(team_sizes_time_6, avg_response_time_6, color='brown', marker='x', linestyle='-', label='PTFO_S_Re_Init')
plt.plot(team_sizes_time_7, avg_response_time_7, color='black', marker='^', linestyle='-', label='TFO_Re')



# Add labels and title
plt.xlabel('Number of Agents', fontsize=20)
plt.ylabel('Average Response Time (seconds)', fontsize=20)
# plt.title(f'{map_name}', fontsize=20, fontweight='bold')

# Show grid, legend, and make it clear which algorithm is which
plt.grid(True)
# plt.legend()

# Display the plot
plt.show()