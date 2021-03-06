import carla
def annotate_me(waypoints, ro = True):
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    if ro:
        for w, _ in waypoints:
            x, y = w.transform.location.x, w.transform.location.y
            world.debug.draw_string(w.transform.location, f"{x=} {y=}", draw_shadow=False,
                                            color=carla.Color(r=255, g=0, b=0), life_time=360.0,
                                            persistent_lines=True)
    else:
        for w in waypoints:
            x, y = w.transform.location.x, w.transform.location.y
            world.debug.draw_string(w.transform.location, f"{x=} {y=}", draw_shadow=False,
                                            color=carla.Color(r=255, g=0, b=0), life_time=360.0,
                                            persistent_lines=True)

if __name__ == "__main__":
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    world_map = world.get_map()
    waypoints = world_map.generate_waypoints(10)
    annotate_me(waypoints, False)
