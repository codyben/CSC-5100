import carla
import time, random
from sync_mode import CarlaSyncMode
from HumanClient import ProjectClient as Human
from AVClient import ProjectClient as AV

random.seed(5100)

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10.0)
world = client.get_world()

init_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
# After that, set the TM to sync mode
my_tm = client.get_trafficmanager()
my_tm.set_synchronous_mode(True)
world.apply_settings(init_settings)
try:
    with CarlaSyncMode(world) as w:

        h = Human(world = w.world)
        a = AV(world = w.world)
        while True:
            w.tick(9999)
except:
    pass
finally:
    pass