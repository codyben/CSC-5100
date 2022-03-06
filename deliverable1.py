
import carla
from agents.navigation.controller import VehiclePIDController
import random
# To import a behavior agent
from agents.navigation.behavior_agent import BehaviorAgent

from multiprocessing import Pool

def agent_control(t):
  agent, vehicle = t
  while True:
    print("RUNNING")
    vehicle.apply_control(agent.run_step())


client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
world_map = world.get_map()
waypoint02 = world_map.get_waypoint_xodr(60,-6,0)
spawn_points = world.get_map().get_spawn_points()
vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]

AGENTS = []
for _ in range(1):
  s = random.choice(spawn_points)
  vehicle = world.spawn_actor(vehicle_blueprint, s)
  agent = BehaviorAgent(vehicle)
  agent.set_target_speed(80)

  agent.set_destination(waypoint02.transform.location)
  # print(waypoint02.transform)
  # draw_waypoints([waypoint02], world)
  
  while True:
    if agent.done():
      print("ALL DONE")
      break
    print("CONTROL")
    vehicle.apply_control(agent.run_step())

  
