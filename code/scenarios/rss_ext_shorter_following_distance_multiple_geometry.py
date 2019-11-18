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

# Shorter following distance multiple geometry.
# The ego vehicle enters a roundabout, maintaining a safe following distance
# behind a lead vehicle.
# Variants:
#      a: Current RSS library does not yet include roundabouts and ego
#         may collide with lead vehicle.
#      b: Ego vehicle enters roundabout at RSS Classic safe following
#         distance.
#      c: RSS Extended ego vehicle sets limits on its own acceleration
#         and braking to achieve a shorter following distance.
class RssExtShorterFollowingDistanceMG(BasicScenario):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, variant="RSS_Ext_SFDMG_a"):

        self._rss_params = rss_params
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._other_actor_target_speed = config.other_actors[0].target_speed
        self._target = config.target.transform.location

        self._filename = filename
        self.timeout=60
        self._map = CarlaDataProvider.get_map()
        self._variant = variant
        self._world = world 

        super(RssExtShorterFollowingDistanceMG, self).__init__("RssExtShorterFollowingDistanceMG",
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

        self._other_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z),
            config.other_actors[0].transform.rotation)
        vehicle = self._world.try_spawn_actor(vehicle_bp, self._other_transform)
        vehicle.set_simulate_physics(enabled=True)
        self.other_actors.append(vehicle)

    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")

    def _create_behavior(self):
        """
        Shorter following distance multiple geometry behavior.
        The lead vehicle drives for 60 meters, passing through a roundabout.
        The ego vehicle follows the lead vehicle for a portion of the roundabout.
        """

        other_drive = py_trees.composites.Parallel("Other Driving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        other_drive.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_speed))
        other_drive.add_child(DriveDistance(self.other_actors[0], 60))

        ego_drive = py_trees.composites.Sequence("Ego Driving")
        ego_drive.add_child(RssExtBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, self._target, variant=self._variant))

        parallel_drive = py_trees.composites.Parallel("All Cars Driving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(other_drive)
        parallel_drive.add_child(ego_drive)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("All Behavior Sequence")

        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_transform))
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
