# Start-Kit

## Flexibility-Based Traffic Flow Optimisation in Lifelong Multi-Agent Path Finding
The repository contains the implementation of the paper: Peiqian Lin, Zhe Chen, David L. Dowe, Daniel Harabor, Flexibility-Based Traffic Flow Optimisation in Lifelong Multi-Agent Path Finding. The paper was accepted for publication at AAMAS 2026. https://doi.org/10.65109/IRBO1733

## PTFO
This branch is the implementation of PTFO. In this setting, we set S = A and impose no time limit on each timestep. In other words, all agents update their OCPGs and contribute traffic flows to the overall traffic flow at each timestep. We also set Refine=false, which means that RefineGuidePaths() is not executed.

## Other versions
PTFO_S: https://github.com/plin0022/Start-Kit/tree/traffic_heuristics_mdd_float_shared_no_fw


### Compiling
Using cmake: 
```shell
mkdir build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j
```

### Run the start kit
Running the start-kit using commands: 
```shell
./build/lifelong --inputFile the_input_file_name --simulationTime maximum_timesteps --preprocessTimeLimit preprocess_timelimit --planTimeLimit timelimit_per_timestep -o output_file_location
```

for example:
```shell
./build/lifelong --inputFile ./example_problems/benchmark-lifelong/sortation_small_0_800.json --simulationTime 500 --preprocessTimeLimit 100000 --planTimeLimit 1000 -o test.json
```

more info on help:
```shell
./build/lifelong --help
```

### An example script for running experiments
```shell
import subprocess
import os


# Path to the algorithm
executable_path = './algorithm/lifelong'


# Path to the directory containing your .json files
input_dir = './example_problems/benchmark-lifelong/'


# Path to the output directory
output_dir = './outputs/'


# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)


# The map prefix and simulation timestep
map_prefix, simulation_time = 'sortation_small', '500'

# map_prefix, simulation_time = 'room-64-64-8', '500'

# map_prefix, simulation_time = 'ost003d', '1940'

# map_prefix, simulation_time = 'warehouse_large', '3200'


# The preprocess time (this is not included in the runtime. 
# However, when there are more than 10,000 agents, initializing 
# space for all agents may exceed system limits and cause the program to stop.)
preprocess_time_limit = '100000'


# The plan time limit (no time limit for each step in PTFO)
plan_time_limit = '1000'


# Loop through all .json files in the input directory
for input_file in os.listdir(input_dir):
    if input_file.startswith(map_prefix) and input_file.endswith('.json'):
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

```

## Notes
You might need the following content since our work is based on the start-kit

## ---------------------------------------------------------    

## Join the competition

Log in to the [competition website](http://www.leagueofrobotrunners.org/) with a GitHub account, and we will automatically create a private GitHub submission repo for you.
The repo will be the place where you submit codes. In the `My Submission` page, you can click "My Repo" to open your GitHub submission repo page.

## Clone your submission repo

Clone your submission repo to your local machine. The repo contains starter codes to help you prepare your submission.

```
$ git clone git@github.com:your_submission_repo_address
$ cd your_submission_repo
```

## Compile the start-kit

### Dependencies

- [cmake >= 3.16](https://cmake.org/)
- [libboost >= 1.49.0](https://www.boost.org/)
- Python3 >= 3.11 and [pybind11](https://pybind11.readthedocs.io/en/stable/) >=2.10.1 are recommanded for python interface user.

Install dependencies on Ubuntu or Debian Linux:
```shell
sudo apt-get update
sudo apt-get install build-essential libboost-all-dev python3-dev python3-pybind11 
```

[Homebrew](https://brew.sh/) is recomanded for installing dependencies on Mac OS.

### Compiling

Using `compile.sh`:
```shell
./compile.sh
```

Using cmake: 
```shell
mkdir build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j
```

## Run the start kit

Running the start-kit using commands: 
```shell
./build/lifelong --inputFile the_input_file_name -o output_file_location
```

for example:
```shell
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json
```

more info on help:
```shell
./build/lifelong --help
```

## Windows users
If you are a Windows user, the most straightforward method to utilize our start-kits is by employing the WSL (Windows Subsystem for Linux) subsystem. Follow these steps:
1. Install WSL, please refer to [https://learn.microsoft.com/en-us/windows/wsl/install](https://learn.microsoft.com/en-us/windows/wsl/install)
2. Open a shell in WSL and execute the following commands to install the necessary tools (CMake, GCC, Boost, pip, Pybind11):
```shell
sudo apt-get update
sudo apt-get install cmake g++ libboost-all-dev python3-dev python3-pip
pip install pybind11-global numpy
```
3. Employ the commands provided above to compile the start-kit.

While it's technically possible to use our start-kit with Cygwin, Mingw, and MSVC, doing so would be more complex compared to using WSL. You would likely need to configure the environment yourself.

If you are a docker user, another choice is to develop and test your python implementation under a docker environment. You can the re-create the evaluation environment locally on your machine. For more details, check out the [Test in Docker](./Prepare_Your_Submission.md#test-in-docker) section.

## Upgrade Your Start-Kit

If your private start-kit copy repo was created before a start-kit upgrade, you could run the script `./upgrade_start_kit.sh` to upgrade your start-kit to the latest version.

You can check `version.txt` to know the current version of your start-kit.

The `upgrade_start_kit.sh` will check which file is marked as an upgrade needed and pull those files from the start-kit. It will pull and stage the files, but not commit them. This allows you to review the changes before committing them. 

For files stated as unmodifiable in [Parepare_Your_Planner.md](./Prepare_Your_Submission.md), you always commit their changes.

⚠️ But please be aware that, the start-kit v2.1.0 introduces requested API changes on `task_pool`. This requires minor revision to your implementation to adapt to the new API.  
This change also impacts the implementation of function `update_goal_locations` in `src/Entry.cpp`, therefore, the upgrade script will pull the new version of `src/Entry.cpp` and may overwrite your changes. You could compare the difference using `git diff` and decide whether to revert some modifications or partially accept changes on this file. 

The upgrade script will not touch most of the participants' implementation file.
How every the example implementation in `python/pyMAPFPlanner.py`,`python/pyTaskScheduler.py`, `inc/MAPFPlanner.h`, `inc/TaskScheduler.h`, `src/MAPFPlanner.cpp`, `src/TaskScheduler.cpp`, `default_planner/planner.cpp` and `default_planner/scheduler.cpp` are updated with with new API and additional documentaion. You may want to view changes on these files. 

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Submission.md](./Prepare_Your_Submission.md).

## Debug and Visualise Your Planner
We provide a visualisation tool written in Python: [https://github.com/MAPF-Competition/PlanViz](https://github.com/MAPF-Competition/PlanViz).
It is able to visualise the output of the start-kit program and help participants debug the implementations. 

Please refer to the project website for more information. Also the document [Debug_and_Visualise_Your_Planner](./Debug_and_Visualise_Your_Planner.md) which provides helpful hints for interpreting and diagnosing planner output.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).



