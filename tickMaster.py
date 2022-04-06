import carla
import time, random, sys, weakref
from sync_mode import CarlaSyncMode
from HumanClient import ProjectClient as Human
from AVClient import ProjectClient as AV
from helper import AllRouteCompletedException, log_me

random.seed(5100)
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10.0)
world = client.get_world()

AV_DONE = False
HUMAN_DONE = False

settings = world.get_settings()
settings.synchronous_mode = True
settings.no_rendering_mode = True
settings.fixed_delta_seconds = 0.05
# After that, set the TM to sync mode
my_tm = client.get_trafficmanager()
my_tm.set_synchronous_mode(True)
world.apply_settings(settings)
snapshot = None
try:
    log_me("Beginning simulation.")
    with CarlaSyncMode(world) as w:
        weak_world = weakref.ref(w.world) # pass a reference to synchronized world
        human = Human(world = weak_world)
        av = AV(world = weak_world)
        while not(HUMAN_DONE and AV_DONE):
            try:
                if not HUMAN_DONE:
                    human.tick_me(None)
            except AllRouteCompletedException:
                log_me("Human vehicles completed.")
                HUMAN_DONE = True
            
            try:
                if not AV_DONE:
                    av.tick_me(None)
            except AllRouteCompletedException:
                log_me("Autonomous vehicles completed.")
                AV_DONE = True

            w.tick(9999) # maybe we can take some data from the snapshots?
    log_me("Simulation completed.")
except Exception as e:
    log_me(f"Caught exception: {str(e)}")
    sys.exit(1)
finally:
    log_me("Exiting")