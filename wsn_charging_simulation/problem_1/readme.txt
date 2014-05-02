Data:
- All the data are .csv files.

- The name of a data file starts with "data_one_center", and the following part indicates the purpose of this data file. For example, "data_one_center_base_distance.csv" is the data for testing different distance of base station.

- Each .csv file has three columns: configuration of the experiment, algorithm name, round number

- For the details of the configuration of the experiment column, check out the variable "network_type_str" in start_experiment.py



Figures:
- For each type of data, there is a corresponding script for generating figures.

- The name of a figure generating script file starts with "generate_figure_one", and the following part indicates the purpose of this script.

- For example, "generate_figure_one_base_distance.py" is the script to generate figures based on "data_one_center_base_distance.csv".

- All the scripts need the library, matplotlib. The common functions for generating figures are located in "generate_figure_common.py".



Simulation:
- Components:
-- object_manager.py
-- UAV_AI
-- start_experiments.py
-- start_visualization.py
-- test_config.py

- Usage:
-- Initially all the system parameters are specified in "test_config.py", and you can initialize some of them in the top of  "start_experiments.py" based on the experiment purpose. 
-- "param_network_type" has two available options, "homogeneous" and "homogeneous2". "homogeneous" means that all the sensor nodes have the same initial energy and capacity. "homogeneous2" means that all the sensor node have the same capacity but different initial energy (See object_manager.py for details).
-- The testing algorithms have two parts, "param_charge_mode" and "param_path_mode". The system will automatically combines these two lists and filter out invalid combinations.
-- Run "python start_experiments.py" to start a simulation.
-- You are going to see some code for testing multi-flights, which is not completely implemented yet.

- Visualization:
-- Do not run visualization for experiments. It is too slow. Do use it for debugging.
-- Specify the values of the parameters in the top of "start_visualization.py" and "vis_config.py".
-- Run "python start_visualization.py" to start the visualization.

- Others:
-- Available charging modes: ['random', 'to_full', 'to_initial_average', 'with_constant', 'to_optimized_one_flight']
-- Available path planning modes: ['least_power', 'closest', 'hamiltonian']



Future Development:
- Multi-Flights: The current code should be able to be extended to multi-flights with very few efforts. The problem is that the current simulation is based on the second (it simulates the system state of every second), and thus it is going to need a lot of time if we need to simulate a time period of months. One idea is to accelerate the simulation of sensor nodes when the UAV is not working, considering that each sensor node has a fixed consumption rate.

- Multi-UAVs: The current code can be extended to simulate a centralized multi-UAVs system. I would suggest to rewrite the system if you need to simulate a distributed multi-UAVs system.