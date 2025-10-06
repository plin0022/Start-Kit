import json
import os
import matplotlib.pyplot as plt
from collections import defaultdict

# map name


map_name = 'warehouse_large'


# Path to the directories containing the output files for each algorithm
output_dir_algorithm_1 = f'./{map_name}/traffic_mdd_float_testing_sampling_30'
output_dir_algorithm_2 = f'./{map_name}/traffic_paper_testing'
output_dir_algorithm_3 = f'./{map_name}/pibt_testing'
output_dir_algorithm_4 = f'./{map_name}/traffic_mdd_float_testing_sampling_30_fw'
output_dir_algorithm_5 = f'./{map_name}/traffic_paper_testing_fw'



# Function to extract and average data by team size
def extract_and_average_data(output_dir):
    team_size_to_tasks = defaultdict(list)

    for output_file in os.listdir(output_dir):
        if output_file.endswith('_output.json'):
            output_file_path = os.path.join(output_dir, output_file)

            with open(output_file_path, 'r') as file:
                data = json.load(file)

            # Extract teamSize and numTaskFinished
            team_size = data.get('teamSize')
            num_task_finished = data.get('numTaskFinished')

            if team_size is not None and num_task_finished is not None:
                team_size_to_tasks[team_size].append(num_task_finished)

    # Compute the average numTaskFinished for each team size
    avg_results = {team_size: sum(tasks) / len(tasks) for team_size, tasks in team_size_to_tasks.items()}
    return sorted(avg_results.items())  # Return sorted by teamSize


# Get averaged data for each algorithm
avg_data_1 = extract_and_average_data(output_dir_algorithm_1)
avg_data_2 = extract_and_average_data(output_dir_algorithm_2)
avg_data_3 = extract_and_average_data(output_dir_algorithm_3)
avg_data_4 = extract_and_average_data(output_dir_algorithm_4)
avg_data_5 = extract_and_average_data(output_dir_algorithm_5)



# Extract x (team sizes) and y (average numTaskFinished) for plotting
team_sizes_1, avg_num_tasks_1 = zip(*avg_data_1) if avg_data_1 else ([], [])
team_sizes_2, avg_num_tasks_2 = zip(*avg_data_2) if avg_data_2 else ([], [])
team_sizes_3, avg_num_tasks_3 = zip(*avg_data_3) if avg_data_3 else ([], [])
team_sizes_4, avg_num_tasks_4 = zip(*avg_data_4) if avg_data_4 else ([], [])
team_sizes_5, avg_num_tasks_5 = zip(*avg_data_5) if avg_data_5 else ([], [])


# Plot the relationship between teamSize and avg numTaskFinished for all three algorithms
plt.figure(figsize=(10, 6))


plt.plot(team_sizes_1, avg_num_tasks_1, color='b', marker='o', linestyle='-', label='PTFO_Sa30')
plt.plot(team_sizes_2, avg_num_tasks_2, color='g', marker='^', linestyle='-', label='TFO')
plt.plot(team_sizes_3, avg_num_tasks_3, color='purple', marker='s', linestyle='-', label='PIBT')
plt.plot(team_sizes_4, avg_num_tasks_4, color='pink', marker='o', linestyle='-', label='PTFO_Sa30_Re') 
plt.plot(team_sizes_5, avg_num_tasks_5, color='black', marker='^', linestyle='-', label='TFO_Re') 




# Add labels and title
# plt.xlabel('Team Size')
plt.ylabel('Average Number of Tasks Finished', fontsize=20)
plt.title(f'{map_name}', fontsize=25, fontweight='bold')




# Show grid, legend, and make it clear which algorithm is which
plt.grid(True)
# plt.legend()

# Display the plot
plt.show()




