import json
import os
import matplotlib.pyplot as plt

# Path to the two directories containing the output files for each algorithm
output_dir_algorithm_1 = './outputs/traffic_heuristic'  # Update this path
output_dir_algorithm_2 = './outputs/traffic_paper'  # Update this path
output_dir_algorithm_3 = './outputs/pibt'  # Update this path

# Lists to store teamSize and numTaskFinished values for both algorithms
team_sizes_1 = []
num_tasks_finished_1 = []
team_sizes_2 = []
num_tasks_finished_2 = []
team_sizes_3 = []
num_tasks_finished_3 = []

# Function to extract data from output files
def extract_data_from_directory(output_dir, team_sizes, num_tasks_finished):
    for output_file in os.listdir(output_dir):
        if output_file.endswith('_output.json'):
            output_file_path = os.path.join(output_dir, output_file)
            
            # Read the JSON content from the file
            with open(output_file_path, 'r') as file:
                data = json.load(file)
            
            # Extract teamSize and numTaskFinished from the data
            team_size = data.get('teamSize')
            num_task_finished = data.get('numTaskFinished')
            
            if team_size is not None and num_task_finished is not None:
                team_sizes.append(team_size)
                num_tasks_finished.append(num_task_finished)

# Extract data for both algorithms
extract_data_from_directory(output_dir_algorithm_1, team_sizes_1, num_tasks_finished_1)
extract_data_from_directory(output_dir_algorithm_2, team_sizes_2, num_tasks_finished_2)
extract_data_from_directory(output_dir_algorithm_3, team_sizes_3, num_tasks_finished_3)

# Plot the relationship between teamSize and numTaskFinished for both algorithms
plt.figure(figsize=(10, 6))

# Plot data for Algorithm 1 (using blue color and 'o' marker)
plt.scatter(team_sizes_1, num_tasks_finished_1, color='b', label='traffic_heuristic', marker='o')

# Plot data for Algorithm 2 (using red color and 'x' marker)
plt.scatter(team_sizes_2, num_tasks_finished_2, color='r', label='traffic_paper', marker='x')

# Plot data for Algorithm 3 (using green color and '^' marker)
plt.scatter(team_sizes_3, num_tasks_finished_3, color='g', label='pibt', marker='^')

# Add labels and title
plt.xlabel('Team Size')
plt.ylabel('Number of Tasks Finished')
plt.title('Team Size - Number of Tasks Finished')

# Show grid, legend, and make it clear which algorithm is which
plt.grid(True)
plt.legend()

# Display the plot
plt.show()