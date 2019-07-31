import carla
import py_trees

from scenario_runner_extension.rss_basic_agent import RssBasicAgent

from srunner.scenariomanager.atomic_scenario_behavior import AtomicBehavior, calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


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

        location = CarlaDataProvider.get_location(self._actor)
        if calculate_distance(location, self._target_location) < self._acceptable_target_distance:
            new_status = py_trees.common.Status.SUCCESS

        self._control = self._agent.run_step()
        self._actor.apply_control(self._control)

        return new_status

    def terminate(self, new_status):
        self._control.throttle = 0.0
        self._control.brake = 0.0
        self._actor.apply_control(self._control)
        super(RssBasicAgentBehavior, self).terminate(new_status)