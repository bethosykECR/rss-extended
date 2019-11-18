import carla
import py_trees

from scenario_runner_extension.rss_ext_behavior import RssExtBehavior
from scenario_runner_extension.rss_criteria import RssTest

from srunner.tools.scenario_helper import *
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *

# Parking lot scenario
# The ego vehicle drives out of a parking lot
class RssExtParkingLot(BasicScenario):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, variant="Rss_Ext_PARKLOT_a"):

        self._rss_params = rss_params
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._target = config.target.transform.location

        self._filename = filename
        self.timeout = 60
        self._map = CarlaDataProvider.get_map()
        self._variant = variant
        self._world = world

        super(RssExtParkingLot, self).__init__("RssExtParkingLot",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")

    def _create_behavior(self):
        """
        Parking lot behavior
        The ego vehicle drives out of a parking lot towards a destination
        outside of the parking lot.  The scenario ends after the ego vehicle
        has driven 50 meters.
        """

        ego_drive = py_trees.composites.Parallel("Ego Driving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_drive.add_child(RssExtBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, self._target, variant=self._variant))
        ego_drive.add_child(DriveDistance(self.ego_vehicles[0], 50))

        # Build behavior tree
        sequence = py_trees.composites.Sequence("All Behavior Sequence")

        sequence.add_child(TimeOut(1))
        sequence.add_child(ego_drive)

        return sequence


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename)
        criteria.append(rss_criterion)
        return criteria
