#############################
# Scenario Runner Mode
# Run scenario runner and attach an agent to the ego vehicle 
# Scenario should be executed separately before running this script 
#############################

export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/op_ros2_agent.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="20"
# Autonomous actor default role_name
export AGENT_ROLE_NAME="hero"

# modes are 
#   * "leaderboard" : when runner the leaderboard (route based) scenario collections 
#   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
#   * "free" : when loading empty map only , either carla town or any OpenDRIVE map 
export OP_BRIDGE_MODE="srunner" 

# CARLA town name or custom OpenDRIVE absolute path, when BRIDGE_MODE is free 
export FREE_MAP_NAME="Town01" 

# Spawn point for the autonomous agent, when BRIDGE_MODE is free 
# "x,y,z,roll,pitch,yaw"
# Empty string means random starting position
#export FREE_AGENT_POSE="175.4,195.14,0,0,0,180" 
export FREE_AGENT_POSE=""

gnome-terminal -- bash -c roscore

python ${OP_BRIDGE_ROOT}/op_bridge/op_bridge.py
