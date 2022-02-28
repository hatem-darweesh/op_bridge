# op_bridge - CARLA OpenPlanner Bridge
OpenPlanner ROS based bridge for CARLA Simulator and Scenario Runner 

## Support: 
- CARLA Simulator 0.9.13 release 
- Python 2.7 
- ROS 1
- OpenPlanner.1.13 
- Linux 18.04
- To use traffic light detector: CUDA 11.1 + cudnn 8.0.4

Environment setup (bashrc): 
```
export CARLA_ROOT=/home/user/carla-0.9.13/CARLA_0.9.13_RSS
export SCENARIO_RUNNER_ROOT=/home/user/carla-0.9.13/scenario_runner
export LEADERBOARD_ROOT=/home/user/carla-0.9.13/op_bridge
export TEAM_CODE_ROOT=/home/user/carla-0.9.13/op_agent
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/util
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg
```

How to run the bridge: 

1-Run CARLA simulator 0.9.13 

2-Run one of the following options. 

3-In case of run_srunner_agent.sh, scneario must be started before running the bridge. 

4-The bridge run script will run the op_agent

Three options: 
```
./op_scripts/run_exploration_mode.sh
./op_scripts/run_srunner_agent.sh
./op_scripts/run_route_scenarios.sh
```
