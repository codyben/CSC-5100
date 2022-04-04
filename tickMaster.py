import carla
import time

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
counter = 0
while True:
    time.sleep(0.2) # 5 fps, so 1/0.2 = 5
    world.tick()
    if counter % 2 == 0:
        print("tick")
    else:
        print("tock")
    counter += 1