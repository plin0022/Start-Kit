import subprocess
import os


# Path to the algorithm

# executable_path = './algorithm/lifelong_traffic_paper'

# executable_path = './algorithm/lifelong_traffic_mdd_float'

# executable_path = './algorithm/lifelong_traffic_mdd_float_shared'

# executable_path = './algorithm/lifelong_traffic_mdd_float_shared_fw'

executable_path = './algorithm/lifelong_traffic_mdd_float_sampling_30_fw'





# Path to the directory containing your .json files
input_dir = './example_problems/benchmark-lifelong/'

# Path to the output directory
output_dir = './outputs/'

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)


# The map prefix and simulation timestep

# map_prefix, simulation_time = 'sortation_small', '500'

# map_prefix, simulation_time = 'room-64-64-8', '500'

# map_prefix, simulation_time = 'ost003d', '1940'

map_prefix, simulation_time = 'warehouse_large', '3200'



# The preprocess time
preprocess_time_limit = '1000000'



# The plan time limit

# plan_time_limit = '1000000'

plan_time_limit = '1000'  # frank_wolfe



# Loop through all .json files in the input directory
for input_file in os.listdir(input_dir):

    # if (
    # input_file.startswith(map_prefix) and
    # any(suffix in input_file for suffix in ['_0_']) and
    # input_file.endswith('.json')
    # ):


    # if (
    # input_file.startswith(map_prefix) and
    # any(suffix in input_file for suffix in ['_2_','_3_','_4_','_5_','_6_','_7_','_8_','_9_']) and
    # input_file.endswith('.json')
    # ):


    # if (
    # input_file.startswith(map_prefix) and
    # any(suffix in input_file for suffix in ['_1_','_2_','_3_','_4_']) and
    # input_file.endswith('.json')
    # ):


    if (
    input_file.startswith(map_prefix) and
    any(suffix in input_file for suffix in ['_5_','_6_','_7_','_8_','_9_']) and
    input_file.endswith('.json')
    ):


    # if (
    # input_file.startswith(map_prefix) and
    # any(suffix in input_file for suffix in ['_5_','_6_','_7_','_8_','_9_','_10_','_11_','_12_','_13_','_14_','_15_',
    # '_16_','_17_','_18_','_19_','_20_','_21_','_22_','_23_','_24_']) and
    # input_file.endswith('.json')
    # ):




    # if (
    # input_file.startswith(map_prefix) and
    # any(suffix in input_file for suffix in ['_0_6000','_0_8000','_0_10000','_0_12000','_0_14000']) and
    # input_file.endswith('.json')
    # ):



    # if input_file.startswith(map_prefix) and input_file.endswith('.json'):
    


        # Construct the full path for the input file
        input_file_path = os.path.join(input_dir, input_file)
        
        # Define the output file name and place it in the output directory
        output_file = os.path.join(output_dir, input_file.replace('.json', '_output.json'))
        
        # arguments
        args = [
            executable_path,
            '--inputFile', input_file_path, 
            '-o', output_file,
            '--simulationTime', simulation_time,
            '--preprocessTimeLimit', preprocess_time_limit,
            '--planTimeLimit', plan_time_limit
        ]
        
        # Run the command
        result = subprocess.run(args, capture_output=True, text=True)

        # Check if the command is successful
        if result.returncode == 0:
            print(f"Execution successful for {input_file}")
            print("Output:", result.stdout)
        else:
            print(f"Execution failed for {input_file}")
            print("Error:", result.stderr)
