
#openplanner bridge
# Hatem Darweesh January 2022 

import math
import signal
import os
import sys
import importlib
import time
import traceback
import carla
from srunner.scenariomanager.carla_data_provider import *
from leaderboard.autoagents.agent_wrapper import  AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.result_writer import ResultOutputProvider



class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter('*vehicle*')
        for car in actors:
            if car.attributes['role_name'] == role_name:
                return car
        return None

class AgentLoop(object):
    
    def __init__(self):
        self.start_game_time = None
        self.start_system_time = None
        self.debug_mode = False
        self.agent = None
        self.ego_vehicle = None
        self.running = False
        self.timestamp_last_run = 0.0
        self.timeout = 20.0
        self.role_name = 'hero'
        watchdog_timeout = max(5, self.timeout - 2)        
        agent_timeout = watchdog_timeout - 1
        self._agent_watchdog = Watchdog(agent_timeout)

    def _stop_loop(self):
        self.running = False


    def _tick_agent(self, timestamp):                
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds
            
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            try:
                ego_action = self.agent()
            
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            except Exception as e:
                raise AgentError(e)

            self.ego_vehicle.apply_control(ego_action)            
            
            if BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.role_name) is None:
                self.running = False

            spectator = CarlaDataProvider.get_world().get_spectator()
            ego_trans = self.ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=50),
                                                        carla.Rotation(pitch=-90)))

        if self.running:
            CarlaDataProvider.get_world().tick()

class WorldHandler(object):
    def __init__(self):
        self._local_host = os.environ['SIMULATOR_LOCAL_HOST']
        self._port = int(os.environ['SIMULATOR_PORT'])
        self._frame_rate = float(os.environ['AGENT_FRAME_RATE'])
        self._agent_role_name = os.environ['AGENT_ROLE_NAME']
        self._bridge_mode = os.environ['OP_BRIDGE_MODE']
        self._map_name = os.environ['FREE_MAP_NAME']
        self._spawn_point = os.environ['FREE_AGENT_POSE']
        self._world = None
    
    def load_world(self):
        client = carla.Client(self._local_host, self._port)
        client.set_timeout(2)    
        
        if self._bridge_mode == 'free' or self._bridge_mode == '':
            client.load_world(self._map_name)                
        
        self._world = client.get_world()
        if self._world is not None:
            settings = self._world.get_settings()
            settings.fixed_delta_seconds = 1.0 / self._frame_rate
            settings.synchronous_mode = True
            self._world.apply_settings(settings)
            CarlaDataProvider.set_world(self._world)
            CarlaDataProvider.set_client(client)            

            if self._bridge_mode == 'free' or self._bridge_mode == '':                    
                spawn_point = carla.Transform()
                point_items = self._spawn_point.split(',')
                _randomize = False
                if len(point_items) == 6:
                    spawn_point.location.x = float(point_items[0])
                    spawn_point.location.y = float(point_items[1])
                    spawn_point.location.z = float(point_items[2]) + 2  
                    spawn_point.rotation.roll = float(point_items[3])
                    spawn_point.rotation.pitch = float(point_items[4])
                    spawn_point.rotation.yaw = float(point_items[5])
                else:
                    _randomize = True                
            
                CarlaDataProvider.request_new_actor('vehicle.audi.tt', spawn_point, self._agent_role_name, random_location=_randomize)            
        else:
            print("Can't Load CARLA .. make sure that Simulator is running !!")  

    def _cleanup(self):
        CarlaDataProvider.cleanup()                    

class AgentHandler(object):
    def __init__(self, world_handler):
        self._agent_role_name = os.environ['AGENT_ROLE_NAME']
        agent_path = os.environ['TEAM_AGENT']    
        module_name = os.path.basename(agent_path).split('.')[0]    
        sys.path.insert(0, os.path.dirname(agent_path))    
        module_agent = importlib.import_module(module_name)    
        agent_class_name = getattr(module_agent, 'get_entry_point')()
        self.agent_instance = getattr(module_agent, agent_class_name)('')
        self._agent_wrapper = AgentWrapper(self.agent_instance)
        self.ego_vehicle = BridgeHelpers.get_agent_actor(world_handler._world, self._agent_role_name)
        
        if self.ego_vehicle is not None:
            print("Ego Vehicle: " , self.ego_vehicle.attributes['role_name'])
            self._agent_wrapper.setup_sensors(self.ego_vehicle, False)
            
            
            # # Set the vehicle dynamics (physics)
            # dynamics = self.ego_vehicle.get_physics_control()
            # dynamics.mass = 5000 
            # dynamics.drag_coefficient = 0.2
            # dynamics.clutch_strength = 1.0
            # dynamics.use_sweep_wheel_collision = 0
            # dynamics.max_rpm = 0
            # dynamics.damping_rate_full_throttle = 0.01
            # front_left_wheel = dynamics.wheels[0]
            # front_right_wheel = dynamics.wheels[1]
            # rear_left_wheel = dynamics.wheels[2]
            # rear_right_wheel = dynamics.wheels[3]

            # front_left_wheel.tire_friction = tire_friction
            # front_left_wheel.long_stiff_value = long_stiff
            # front_left_wheel.lat_stiff_value  = lat_stiff
            # front_left_wheel.lat_stiff_max_load  = lat_load

            # torque_curve (list(carla.Vector2D))
# Curve that indicates the torque measured in Nm for a specific RPM of the vehicle's engine.
# max_rpm (float)
# The maximum RPM of the vehicle's engine.
# moi (float - kg*m2)
# The moment of inertia of the vehicle's engine.
# damping_rate_full_throttle (float)
# Damping ratio when the throttle is maximum.
# damping_rate_zero_throttle_clutch_engaged (float)
# Damping ratio when the throttle is zero with clutch engaged.
# damping_rate_zero_throttle_clutch_disengaged (float)
# Damping ratio when the throttle is zero with clutch disengaged.
# use_gear_autobox (bool)
# If True, the vehicle will have an automatic transmission.
# gear_switch_time (float - seconds)
# Switching time between gears.
# clutch_strength (float - kg*m2/s)
# Clutch strength of the vehicle.
# final_ratio (float)
# Fixed ratio from transmission to wheels.
# forward_gears (list(carla.GearPhysicsControl))
# List of objects defining the vehicle's gears.
# mass (float - kilograms)
# Mass of the vehicle.
# drag_coefficient (float)
# Drag coefficient of the vehicle's chassis.
# center_of_mass (carla.Vector3D - meters)
# Center of mass of the vehicle.
# steering_curve (list(carla.Vector2D))
# Curve that indicates the maximum steering for a specific forward speed.
# use_sweep_wheel_collision (bool)
# Enable the use of sweep for wheel collision. By default, it is disabled and it uses a simple raycast from the axis to the floor for each wheel. This option provides a better collision model in which the full volume of the wheel is checked against collisions.
# wheels (list(carla.WheelPhysicsControl))
# List of wheel physics objects. This list should have 4 elements, where index 0 corresponds to the front left wheel, index 1 corresponds to the front right wheel, index 2 corresponds to the back left wheel and index 3 corresponds to the back right wheel. For 2 wheeled vehicles, set the same values for both front and back wheels.
            
            # wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
            # dynamics.wheels = wheels
            # self.ego_vehicle.apply_physics_control(dynamics)

        else:
            print("Can't Load Ego Vehicle !! Agent will exit ")
            CarlaDataProvider.cleanup()
            self._agent_wrapper.cleanup()
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
            raise Exception("Can't initialize agent ego_vehicle ! ")
    
    def run_agent(self):
        try:    
            self.agent_loop = AgentLoop()
            self.agent_loop.agent = self._agent_wrapper
            self.agent_loop.ego_vehicle = self.ego_vehicle
            self.agent_loop.start_system_time = time.time()
            self.agent_loop.start_game_time = GameTime.get_time()    
            self.agent_loop.role_name = self._agent_role_name            
            self.agent_loop.running = True    
            while self.agent_loop.running:
                timestamp = None
                world = CarlaDataProvider.get_world()
                if world:
                    snapshot = world.get_snapshot()
                    if snapshot:
                        timestamp = snapshot.timestamp
                if timestamp:
                    self.agent_loop._tick_agent(timestamp)                
        except Exception as e:        
            traceback.print_exc()
    
    def _stop_loop(self):
        self.agent_loop._stop_loop()
    
    def _cleanup(self):
        self._agent_wrapper.cleanup()
    
        if self.ego_vehicle:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None                

        if self.agent_instance:                
            self.agent_instance.destroy()
            self.agent_instance = None

def main():
    world_handler = WorldHandler()
    world_handler.load_world()      
    
    agent_handler = AgentHandler(world_handler)

    signal.signal(signal.SIGINT, AgentHandler._stop_loop)

    agent_handler.run_agent()

    print("Scenario Ended , Hero has fallen, Sayonara")  

    world_handler._cleanup()
    agent_handler._cleanup()

if __name__ == '__main__':
    main()
