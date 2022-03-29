# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

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
import random

VIEW_WIDTH = 1920//2
VIEW_HEIGHT = 1080//2
VIEW_FOV = 90

class ProjectClient(object):
    def __init__(self):
        self.client = None
        self.world = None
        self.camera = None
        # self.car = None
        self.agents = []
        self.display = None
        self.image = None
        self.capture = True

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
        origin = carla.Transform(carla.Location(x=-9.746142, y=-180.418823, z=0.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        car = self.world.spawn_actor(car_bp, origin)
        agent = HumanAgent(car, target_speed=random.randint(30,100))
        destinationLocation = carla.Transform(carla.Location(x=-397.648987, y=26.758696, z=0.000000), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        destinationWaypoint = self.world.get_map().get_waypoint(destinationLocation.location)
        self.destination = destinationWaypoint # waypoints[-1]
        print('origin and destination', self.destination.transform.location, origin.location)
        agent.set_destination(self.destination.transform.location, origin.location)
        self.agents.append(agent)
        print(self.agents)

    def setup_camera(self):
        #camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        #self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.agents[0].get_vehicle())
        camera_transform = carla.Transform(carla.Location(x=-9.746142, y=-185.418823, z=5.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))
        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

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

    def run(self):
        try:
            pygame.init()
            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            #self.removeVehicles()
            self.setup_car()
            self.setup_camera()
            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame_clock = pygame.time.Clock()
            self.set_synchronous_mode(True)
            counter = 0
            while True:
                self.world.tick()
                self.capture = True
                pygame_clock.tick_busy_loop(20)
                self.render(self.display)
                pygame.display.flip()
                pygame.event.pump()
                counter = counter + 1
                if counter % 50 == 0:
                    self.setup_car()
                for agent in self.agents:
                    if agent.done():
                        print("The target has been reached, stopping the simulation")
                    agent.get_vehicle().apply_control(agent.run_step())
        finally:
            self.set_synchronous_mode(False)
            self.camera.destroy()
            #self.car.destroy()
            for agent in self.agents:
                agent.get_vehicle().destroy()
            pygame.quit()

try:
    client = ProjectClient()
    client.run()
finally:
    print('EXIT')
