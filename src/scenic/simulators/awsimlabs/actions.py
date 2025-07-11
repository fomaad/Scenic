from scenic.domains.driving.actions import *

class SetDestinationAction(Action):
    def __init__(self, dest):
        self.dest = dest

    def canBeTakenBy(self, agent):
        return True

    def applyTo(self, obj, sim):
        sim.set_ego_destination(obj, self.dest)
        sim.send_engage_cmd(obj)