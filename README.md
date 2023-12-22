# op_bridge - CARLA OpenPlanner Bridge
OpenPlanner ROS based bridge for CARLA Simulator and Scenario Runner 

## Last Update: December 2023, Working on Ubuntu 22.04 and ROS2 humble. Thanks to https://github.com/gezp this is possible. 
- Make sure to Download the Python egg for 3.10 from [here](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04). 
- Add the egg file to the folder: ../CARLA_0.9.15/PythonAPI/carla/dist
- Review the previous [tutorials](https://github.com/orgs/autowarefoundation/discussions/2828)
- When installing the python requirements you might face some issues becuase the python 3.10. Good luck resolving these issue. 

## Support: 
- CARLA Simulator 0.9.15 release 
- Python 3.10 
- ROS2 Humble
- OpenPlanner.Universe
- Linux 22.04

### Environment setup (bashrc): 
```
export AUTOWARE_ROOT=/home/user/carla/autoware
export CARLA_ROOT=/home/user/carla/CARLA_0.9.15
export SCENARIO_RUNNER_ROOT=/home/user/carla/scenario_runner
export OP_BRIDGE_ROOT=/home/user/carla/op_bridge
export OP_AGENT_ROOT=/home/user/carla/op_agent
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/util
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg
```

**How to run the bridge:**
1. Run CARLA simulator 0.9.15 
2. Run one of the following options. 
3. In case of run_srunner_agent.sh, scneario must be started before running the bridge. 
4. The bridge run script will run the op_agent

**Options(ROS2):** 
```
./op_scripts/run_exploration_mode_ros2.sh
./op_scripts/run_srunner_agent_ros2.sh
./op_scripts/run_route_scenarios_ros2.sh
```

**Run the simulator on another PC**
1. Make sure that both the Simulator PC and the Autoware PC are on the same network and you know the IP of each and able to ping both. 
2. In the file "run_exploration_mode_ros2.sh" replace "localhost" with the simulator PC IP address. 
Example: 
```
//export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_LOCAL_HOST="192.168.0.0"
```
