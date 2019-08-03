import carla
import py_trees

from scenario_runner_extension.rss_basic_agent import RssBasicAgent

from srunner.scenariomanager.atomic_scenario_behavior import AtomicBehavior, calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenariomanager.timer import GameTime

class RssBasicAgentBehavior(AtomicBehavior):

    _acceptable_target_distance = 2

    def __init__(self, rss_params, actor, target_location, name="RssBasicAgentBehavior"):
        super(RssBasicAgentBehavior, self).__init__(name)            
        self._agent = RssBasicAgent(actor, rss_params)  # pylint: disable=undefined-variable
        self._agent.set_destination((target_location.x, target_location.y, target_location.z))
        self._control = carla.VehicleControl()
        self._actor = actor
        self._target_location = target_location

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        #location = CarlaDataProvider.get_location(self._actor)
        #if calculate_distance(location, self._target_location) < self._acceptable_target_distance:
        #    print('RSS reached the goal is success')

        self._control = self._agent.run_step()
        self._actor.apply_control(self._control)

        return new_status

    def terminate(self, new_status):
        self._control.throttle = 0.0
        self._control.brake = 0.0
        self._actor.apply_control(self._control)
        super(RssBasicAgentBehavior, self).terminate(new_status)

class WaitForSeconds(AtomicBehavior):

    """
    This class will wait and do nothing for given amount of time 
    """

    def __init__(self, duration, name="WaitForSeconds"):
        """
        Setup actor
        """
        super(WaitForSeconds, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._duration = duration
        self._start_time = 0

    def initialise(self):
        self._start_time = GameTime.get_time()
        super(WaitForSeconds, self).initialise()

    def update(self):

        new_status = py_trees.common.Status.RUNNING

        #print('in WaitForSeconds for %.1fs' % (GameTime.get_time() - self._start_time))
        if GameTime.get_time() - self._start_time > self._duration:
            new_status = py_trees.common.Status.SUCCESS
            #print('I have waited for %.1fs' % self._duration)

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status
