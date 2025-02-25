import subprocess
import os

# Path to your C++ executable in WSL (ensure this is correct)
executable_path = './build/lifelong'

# Path to the directory containing your .json files
input_dir = './example_problems/benchmark-lifelong/'

# Path to the output directory
output_dir = './outputs/'

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

# The map prefix you're interested in (e.g., 'den520d_')
map_prefix = 'sortation_small'

# Loop through all .json files in the input directory
for input_file in os.listdir(input_dir):
    if input_file.endswith('.json') and input_file.startswith(map_prefix):
        # Construct the full path for the input file
        input_file_path = os.path.join(input_dir, input_file)
        
        # Define the output file name and place it in the output directory
        output_file = os.path.join(output_dir, input_file.replace('.json', '_output.json'))
        
        # Use 'wsl' command to ensure it's run in WSL environment
        args = [
            'wsl', executable_path, 
            '--inputFile', input_file_path, 
            '-o', output_file
        ]
        
        # Run the command
        result = subprocess.run(args, capture_output=True, text=True)

        # Check if the command was successful
        if result.returncode == 0:
            print(f"Execution successful for {input_file}")
            print("Output:", result.stdout)
        else:
            print(f"Execution failed for {input_file}")
            print("Error:", result.stderr)
