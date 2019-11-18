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

# Shorter Following Distance
# The ego vehicle follows a lead vehicle.
# By asserting limits on its own acceleration and braking, the RSS Extended ego vehicle obtains a shorter following distance.
#       a:  RSS Classic following distance
#       b:  RSS Extended shorter following distance
class RssExtShorterFollowingDistance(BasicScenario):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, variant="RSS_Ext_SFD_a"):

        self._rss_params = rss_params
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._other_actor_target_speed = config.other_actors[0].target_speed
        self._target = config.target.transform.location

        self._filename = filename
        self.timeout=60
        self._map = CarlaDataProvider.get_map()
        self._variant = variant
        self._world = world

        super(RssExtShorterFollowingDistance, self).__init__("RssExtShorterFollowingDistance",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # Add actors from xml file
        # Make non-ego actors light grey
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter(config.other_actors[0].model))
        vehicle_bp.set_attribute('color', '220,220,220')

        self._lead_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z),
            config.other_actors[0].transform.rotation)
        vehicle = self._world.try_spawn_actor(vehicle_bp, self._lead_transform)
        vehicle.set_simulate_physics(enabled=True)
        self.other_actors.append(vehicle)

    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")

    def _create_behavior(self):
        """
        Shorter Following Distance behavior
        The lead vehicle drives for 100 meters.
        The ego vehicle follows at the RSS Classic or RSS Extended safe
        following distance.
        """

        lead_drive = py_trees.composites.Sequence("Lead Sequence")
        lead_drive.add_child(KeepVelocity(self.other_actors[0], target_velocity=self._other_actor_target_speed, distance=100))

        ego_drive = py_trees.composites.Sequence("Ego Sequence")
        ego_drive.add_child(RssExtBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, self._target, variant=self._variant))

        parallel_drive = py_trees.composites.Parallel("All Cars Driving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(lead_drive)
        parallel_drive.add_child(ego_drive)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("All Behavior Sequence")

        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._lead_transform))
        sequence.add_child(TimeOut(1))
        sequence.add_child(parallel_drive)

        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))
        return sequence


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename)
        criteria.append(rss_criterion)
        return criteria
