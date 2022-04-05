# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
from constants import lane1Spawn, lane2Spawn, lane3Spawn
from constants import humanDestination as _destination

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
import weakref
from humanAgent import HumanAgent
import pygame
import numpy as np
import random, json, time
from datetime import date, datetime, timedelta

VIEW_WIDTH = 1920//2
VIEW_HEIGHT = 1080//2
VIEW_FOV = 90

class ProjectClient(object):
    def __init__(self, world):
        self.world = world
        self.map = self.world.get_map()
        self.vehicle_counter = 0
        self.counter = 0
        self.blueprints = []
        self.blueprintLibrary = self.world.get_blueprint_library()
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.audi.a2')[0])
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.nissan.patrol')[0])
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.vespa.zx125')[0])
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.nissan.micra')[0])
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.kawasaki.ninja')[0])
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.dodge.charger_police')[0])
        self.blueprints.append(self.blueprintLibrary.filter('vehicle.harley-davidson.low_rider')[0])
        self.world.on_tick(lambda s: self.tick_me(s))
        # self.car = None
        self.kill_spawn = False
        self.agent_results = dict()
        self.agents = ()
        self.display = None
        self.image = None
        self.capture = True
        self.collisionDetectors = []
        self.lane3Origin = carla.Transform(carla.Location(**lane3Spawn), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        self.lane2Origin = carla.Transform(carla.Location(**lane2Spawn), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        self.lane1Origin = carla.Transform(carla.Location(**lane1Spawn), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))

    def camera_blueprint(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self, origin):
        #4
        car_bp = random.choice(self.blueprints) #self.world.get_blueprint_library().filter('vehicle.*')[7]
        #origin = carla.Transform(carla.Location(x=-9.746142, y=-180.418823, z=0.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        #origin = carla.Transform(carla.Location(x=-13.0, y=-180.418823, z=0.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        #origin = carla.Transform(carla.Location(x=-17.0, y=-180.418823, z=0.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        originWaypoint = self.map.get_waypoint(origin.location)
        origin = originWaypoint.transform
        car = self.world.spawn_actor(car_bp, origin)
        agent = HumanAgent(car, target_speed=random.randint(30,80)) # adjusted speed down since some cars were veering off road in turns.
        destinationLocation = carla.Transform(carla.Location(**_destination), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        destinationWaypoint = self.map.get_waypoint(destinationLocation.location)
        self.destination = destinationWaypoint # waypoints[-1]
        #print('origin and destination', self.destination.transform.location, origin.location)
        agent.set_destination(self.destination.transform.location, origin.location)
        self.agent_results[agent.get_vehicle().id] = {
            "timing": None,
            "collisions": 0
        }
        self.agents = (*self.agents, agent)
        self.setup_collisionDetection(car)

    def setup_camera(self):
        #camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        #self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.agents[0].get_vehicle())
        camera_transform = carla.Transform(carla.Location(x=-9.746142, y=-195.418823, z=5.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))
        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

    def setup_collisionDetection(self, vehicle):
        collisionDetector = self.world.spawn_actor(self.blueprintLibrary.find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
        collisionDetector.listen(lambda event: self.handleCollision(event))
        self.collisionDetectors.append(collisionDetector)

    def handleCollision(self, event):
        the_car = event.actor
        self.agent_results[the_car.id]['collisions'] += 1

    @staticmethod
    def set_image(weak_self, img):
        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    def render(self, display):
        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))
            
    def removeVehicles(self):
        actor_list = self.world.get_actors()
        for vehicle in actor_list.filter('vehicle.*'):
            vehicle.destroy()

    def run_steps(self, snapshot):
        for agent in self.agents:
            if agent.done():
                print("The target has been reached, stopping the simulation")
                id, timing = agent.destroy_and_time()
                self.agent_results[id]['timing'] = timing
            try:
                agent.get_vehicle().apply_control(agent.run_step())
            except:
                pass
    def attempt_spawn(self, snapshot):
        try: 
            if self.counter % 20 == 0:
                if self.counter % 60 == 0:
                    self.setup_car(self.lane1Origin)
                elif self.counter % 40 == 0:
                    self.setup_car(self.lane2Origin)
                else:
                    self.setup_car(self.lane3Origin)
        except RuntimeError as e:
            # print(f"Caught RuntimeError: {str(e)}")
            pass

    def write_data(self):
        try:
            with open("results.human.json") as f:
                json.dump(self.agent_results, f)
        except:
            pass
            # If we fail to lock the file since another tick callback has it.
    def prune_vehicles(self):
        self.agents = tuple(filter(lambda agent: not agent.done(), self.agents))

    def tick_me(self, snapshot):
        if not self.kill_spawn and len(self.agents) < 15:
            self.attempt_spawn(snapshot)
        else:
           self.kill_spawn = True
        self.prune_vehicles()
        try:
            self.run_steps(snapshot)
        except:
            pass
        if self.kill_spawn and not self.agents:
            self.write_data()
            raise RuntimeError("All vehicles completed.")

    

    
