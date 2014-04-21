Data:
- All the data are .csv files.

- The name of a data file starts with "data_one_center", and the following part indicates the purpose of this data file.

- For example, "data_one_center_base_distance.csv" is the data for testing different distance of base station.



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
-- Initially all the system parameters are specified in "test_config.py", and you can adjust some of them in the top of  "start_experiments.py" based on the experiment purpose. 
-- "param_network_type" has two available options, "homogeneous" and "homogeneous2". "homogeneous" means that all the sensor nodes have the same initial energy and capacity. "homogeneous2" means that all the sensor node have the same capacity but different initial energy (See object_manager.py for details).
-- The testing algorithms have two parts, "param_charge_mode" and "param_path_mode". The system will automatically combines these two lists and filter out invalid combinations.
-- Run "python start_experiments.py" to start a simulation.
-- You are going to see some code for testing multi-flights, which is not completely implemented yet.

- Visualization:
-- Do not run visualization for experiments. It is too slow. Do use it for debugging.
-- Specify the values of the parameters in the top of "start_visualization.py".
-- Run "python start_visualization.py" to start the visualization.