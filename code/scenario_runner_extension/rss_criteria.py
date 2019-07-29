import py_trees

from tools import dist_aux
from tools import other_aux

import sys
sys.path.append('/home/user/alena/scenario_runner')
from srunner.scenariomanager.atomic_scenario_criteria import Criterion

class RssTest(Criterion):

    def __init__(self, actor, filename, optional=False, name="CheckRss", terminate_on_failure=False):
        super(RssTest, self).__init__(name, actor, 0, None, optional, terminate_on_failure)
        world = self.actor.get_world()
        self.vehicles = world.get_actors().filter('vehicle.*')
        self.filename = filename


    def update(self):
        d = dist_aux.evaluate_dist(self.vehicles)
        #print(d)
        other_aux.write2csv(self.filename, d)

        return py_trees.common.Status.RUNNING