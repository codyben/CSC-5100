
import carla
from agents.navigation.controller import VehiclePIDController
import random
# To import a behavior agent
from agents.navigation.behavior_agent import BehaviorAgent
from NormalCar import NormalCar as Car


client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
world_map = world.get_map()
waypoint02 = world_map.get_waypoint_xodr(60,-6,0) #Used this to get coords: https://odrviewer.io/
spawn_points = world.get_map().get_spawn_points()
vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]


s = random.choice(spawn_points)
print(s)

b_currier = lambda b: BehaviorAgent(b, behavior='aggressive')

single_car = Car(vehicle_blueprint, world, s, behavior=b_currier)
single_car.goto(waypoint02)
single_car.drive(lambda : print("Done driving!"))


  
