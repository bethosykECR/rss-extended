import carla
import py_trees

from scenario_runner_extension.rss_basic_agent import RssBasicAgent

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior, calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime

class RssBasicAgentBehavior(AtomicBehavior):

    def __init__(self, rss_params, actor, target_speed, target_location, name="RssBasicAgentBehavior"):
        super(RssBasicAgentBehavior, self).__init__(name)            
        self._target_speed = target_speed * 3.6  # Note: Conversion from m/s to km/h required
        self._actor = actor
        self._rss_params = rss_params
        self._agent = RssBasicAgent(self._actor, self._target_speed, self._rss_params)  # pylint: disable=undefined-variable

        self._agent.set_destination((target_location.x, target_location.y, target_location.z))
        self._control = carla.VehicleControl()
        self._target_location = target_location

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        self._control = self._agent.run_step()
        self._actor.apply_control(self._control)
        return new_status

    def terminate(self, new_status):
        self._control.throttle = 0.0
        self._control.brake = 0.0
        self._actor.apply_control(self._control)
        super(RssBasicAgentBehavior, self).terminate(new_status)