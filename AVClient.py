# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import random
from constants import lane4Spawn
from constants import avDestination as _destination
from annotatemap import annotate_me
from threading import RLock

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
import weakref, json, time
from drivingLaneAgent import DrivingLaneAgent
import pygame
import numpy as np
from helper import AllRouteCompletedException, log_me

VIEW_WIDTH = 1920//2
VIEW_HEIGHT = 1080//2
VIEW_FOV = 90

class ProjectClient(object):
    def __init__(self, world, spawnCap = 6):
        self.spawnCap = spawnCap
        self.world = world()
        self.vehicle_counter = 0
        self.counter = 0
        self.ticks = 0
        self.lock = RLock()
        # weak_self = weakref.ref(self)
        # self.world.on_tick(lambda s: weak_self().tick_me(s))
        # self.car = None
        self.kill_spawn = False
        self.agents = ()
        self.agent_results = dict()
        self.display = None
        self.image = None
        self.capture = True
        self.cameraAttached = False
        self.collisionDetectors = {}
        self.blueprintLibrary = self.world.get_blueprint_library()
        # pygame.init()
        # self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

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

    def setup_car(self):
        car_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
        origin = carla.Transform(carla.Location(**lane4Spawn), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        originWaypoint = self.world.get_map().get_waypoint(origin.location)
        origin = originWaypoint.transform
        car = self.world.spawn_actor(car_bp, origin)
        agent = DrivingLaneAgent(car, 60)
        destinationLocation = carla.Transform(carla.Location(**_destination), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        destinationWaypoint = self.world.get_map().get_waypoint(destinationLocation.location)
        self.destination = destinationWaypoint # waypoints[-1]
        #print('origin and destination', self.destination.transform.location, origin.location)
        agent.set_destination(self.destination.transform.location, origin.location)
        self.agents = (*self.agents, agent)
        self.agent_results[agent.get_vehicle().id] = {
            "timing": None,
            "collisions": 0,
            "ticks": 0
        }
        # weak_self = weakref.ref(self)
        # ProjectClient.setup_collisionDetection(weak_self, agent.get_vehicle())

    def setup_camera(self, car):
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=car)
        camera_transform = carla.Transform(carla.Location(x=-5.746142, y=-185.418823, z=5.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))
        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

    @staticmethod
    def setup_collisionDetection(weak_self, vehicle):
        self = weak_self()
        if not self:
            return
        collisionDetector = self.world.spawn_actor(self.blueprintLibrary.find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
        collisionDetector.listen(lambda event: ProjectClient.handleCollision(self, event))
        self.collisionDetectors[vehicle.id] = collisionDetector

    @staticmethod
    def handleCollision(weak_self, event):
        #https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/manual_control.py

        the_car = event.actor
        weak_self.agent_results[the_car.id]['collisions'] += 1

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
        self.prune_vehicles()
        for agent in self.agents:
            try:
                agent.get_vehicle().apply_control(agent.run_step())
            except:
                pass

    def attempt_spawn(self, snapshot):
        self.counter += 1
        try: 
            if self.counter % 30 == 0:
                self.setup_car()
                    
        except RuntimeError as e:
            # print(f"Caught RuntimeError: {str(e)}")
            pass

    def write_data(self):
        try:
            with open("results.av.json", "w+") as f:
                json.dump(self.agent_results, f)
        except:
            pass
            # If we fail to lock the file since another tick callback has it.
        raise AllRouteCompletedException("Wrote data and completed.")

    def atomic_done_and_remove(self, agent):
        if agent.done():
            id, timing = agent.destroy_and_time()
            log_me(f"AV[{id=}] is done in {self.ticks} ticks")
            self.agent_results[id]['timing'] = timing
            self.agent_results[id]['ticks'] = self.ticks
            return True
        return False

    def prune_vehicles(self):
        self.agents = tuple(filter(lambda agent: not self.atomic_done_and_remove(agent), self.agents))
    
    def tick_me(self, snapshot):
        self.ticks += 1
        if not self.kill_spawn and len(self.agents) < self.spawnCap:
            self.attempt_spawn(snapshot)
        else:
           self.kill_spawn = True
        try:
            self.run_steps(snapshot)
        except Exception as e:
            print(str(e))
        if self.kill_spawn and not self.agents:
            self.write_data()

        # if not self.cameraAttached and self.agents:
        #     self.setup_camera(random.choice(self.agents).get_vehicle())
        #     self.cameraAttached = True
        # pygame.display.flip()
        # pygame.event.pump()

