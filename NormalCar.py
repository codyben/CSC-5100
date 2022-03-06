from agents.navigation.behavior_agent import BehaviorAgent
class NormalCar:
    def __init__(self, blueprint, world, spawn, speed = 80, lane_change=False, behavior=BehaviorAgent):
        self.blueprint = blueprint
        self.world = world
        self.speed = speed
        self.lane_change = lane_change
        self.behavior = behavior
        self.spawn = spawn
        self.vehicle = self.world.spawn_actor(self.blueprint, spawn)
        self.agent = behavior(self.vehicle)
        self.agent.set_target_speed(self.speed)
        self.waypoint_set = False

    def goto(self, waypoint):
        self.agent.set_destination(waypoint.transform.location)
        self.waypoint_set = True

    def done(self):
        return self.agent.done()

    def drive(self, callback=lambda : False):
        if self.agent and self.waypoint_set:
            while not self.done():
                self.vehicle.apply_control(self.agent.run_step())
        callback()
    def force_lane_change(self):
        pass
    