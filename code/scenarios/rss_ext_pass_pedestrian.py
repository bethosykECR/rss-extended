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

# Passing a pedestrian
# The ego vehicle chooses a behavior when a pedestrian crosses a street or
# is close to or walking along the street.
#     a.  Current RSS library does not cover pedestrians and the ego
#         vehicle may collide with a pedestrian.  This corresponds to
#         section 3.8 example 2 where the vehicle has the right-of-way.
#     b.  The ego vehicle is required to yield to a pedestrian in a
#         residential setting, section 3.8 example 1.
#     c.  The ego vehicle is required to remain outisde of a pedestrian's
#         expected route when walking along a residential road, section
#         3.8 example 3.
class RssExtPassPedestrian(BasicScenario):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, variant="RSS_Ext_PASSPED_a"):

        self._rss_params = rss_params
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._target = config.target.transform.location

        self._filename = filename
        self.timeout=60
        self._map = CarlaDataProvider.get_map()
        self._variant = variant
        self._world = world

        super(RssExtPassPedestrian, self).__init__("RssExtPassPedestrian",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # Create a pedestrian.  From CARLA tutorial code:
        # https://github.com/carla-simulator/carla/blob/master/Docs/python_api_tutorial.md
        # and https://github.com/carla-simulator/carla/issues/1229
        blueprintsWalkers = self._world.get_blueprint_library().filter("walker.pedestrian.*")

        # Pedestrians can only spawn at WalkerSpawnPoints.  The UnrealEditor
        # can be used to open a town and locate spawn points.
        ped_location = carla.Location(126.970, 229.260, 1.840)
        spawn_point = carla.Transform(ped_location, carla.Rotation(0))

        self._pedestrian = self._world.try_spawn_actor(random.choice(blueprintsWalkers),spawn_point)
        if (self._pedestrian != None):
            player_control = carla.WalkerControl()
            player_control.speed = 2
            if (self._variant == 'Rss_Ext_PASSPED_c'):
                pedestrian_heading = 170
            else:
                pedestrian_heading=90
            player_rotation = carla.Rotation(0,pedestrian_heading,0)
            player_control.direction = player_rotation.get_forward_vector()
            self._pedestrian.apply_control(player_control)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        self._world.wait_for_tick()


    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")

    def _create_behavior(self):
        """
        Passing a pedestrian behavior.
        The ego vehicle drives a distance of 50 meters, or stops for the
        pedestrian, depending on the scenario variant.
        """

        ego_drive = py_trees.composites.Parallel("Ego Driving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_drive.add_child(RssExtBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, self._target, variant=self._variant))
        if (self._variant == 'Rss_Ext_PASSPED_b'):
            sub_drive = py_trees.composites.Sequence("Ego Stopping")
            sub_drive.add_child(DriveDistance(self.ego_vehicles[0], 20))
            sub_drive.add_child(StopVehicle(self.ego_vehicles[0], self._rss_params['alpha_lon_brake_min']))
            ego_drive.add_child(sub_drive)
        else:
            ego_drive.add_child(DriveDistance(self.ego_vehicles[0], 50))

        # Build behavior tree
        sequence = py_trees.composites.Sequence("All Behavior Sequence")

        sequence.add_child(TimeOut(1))
        sequence.add_child(ego_drive)
        sequence.add_child(ActorDestroy(self._pedestrian))

        return sequence


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename)
        criteria.append(rss_criterion)
        return criteria
