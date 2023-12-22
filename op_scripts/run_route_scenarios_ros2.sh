#############################
# Route based scenario Mode
# Run CARLA leaderboard scenarios and attach an agent to the ego vehicle 
# 
#############################

export SCENARIOS=${OP_BRIDGE_ROOT}/data/all_towns_traffic_scenarios_public.json
export ROUTES=${OP_BRIDGE_ROOT}/data/routes_devtest.xml
export REPETITIONS=1
export DEBUG_CHALLENGE=0
export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/op_ros2_agent.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export CHECKPOINT_ENDPOINT=${OP_BRIDGE_ROOT}/results.json
export CHALLENGE_TRACK_CODENAME=MAP
export AGENT_ROLE_NAME="ego_vehicle"
export OP_BRIDGE_MODE="leaderboard"

gnome-terminal -- bash -c roscore
python3 ${OP_BRIDGE_ROOT}/leaderboard/leaderboard_evaluator.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--record=${RECORD_PATH} \
--resume=${RESUME}
