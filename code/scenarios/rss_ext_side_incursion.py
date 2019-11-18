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

# Side Incursion
# The ego vehicle is traveling forward in a lane.
# A second vehicle protrudes partially into the ego's lane.
# The amount of incursion is flexible.
# In RSS Classic, the ego vehicle will stop since the lateral separation is too small.
# For a small incursion, RSS Extended may allow an ego vehicle to pass while remaining in its own lane.
# For a larger incursion, in RSS Extended, the ego vehicle must change lanes.  An RSS Classic ego vehicle could also choose this behavior.
#
# Variants:
#       a:  Small intrusion.  RSS Classic ego vehicle stops.
#       b:  Small intrusion.  RSS Extended ego vehicle continues in-lane.
#       c:  Large intrusion.  RSS Extended ego vehicle changes lanes.
class RssExtSideIncursion(BasicScenario):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, variant="RSS_Ext_SI_a"):


        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._other_actor_target_speed = config.other_actors[0].target_speed
        self._rss_params = rss_params
        self._target = config.target.transform.location

        self._filename = filename
        self._map = CarlaDataProvider.get_map()
        self.timeout = 60
        self._variant = variant
        self._world = world

        super(RssExtSideIncursion, self).__init__("RssExtSideIncursion",
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
        Side Incursion behavior
        The lead vehicle remains still for 10 seconds and is partially obstructing the ego vehicle's lane.
        The ego vehicle approaches the lead vehicle from behind.
        The scenario ends after ten seconds.
        """

        lead_drive = py_trees.composites.Sequence("Lead Sequence")
        lead_drive.add_child(Idle(duration=10))

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
