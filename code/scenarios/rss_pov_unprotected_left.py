import carla
import py_trees

from scenario_runner_extension.rss_behavior import RssBasicAgentBehavior
from scenario_runner_extension.rss_criteria import RssTest

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *
from srunner.tools.scenario_helper import get_waypoint_in_distance
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *


class RssPovUnprotectedLeft(BasicScenario):
    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        self._rss_params = rss_params
        self._filename = filename
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._other_actor_target_speed = config.other_actors[0].target_speed
        self.timeout=30
        
        super(RssPovUnprotectedLeft, self).__init__("PovUnprotectedLeft", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

        self._ego_traffic_light = CarlaDataProvider.get_next_traffic_light(self.ego_vehicles[0], False)
        self._ego_traffic_light.set_state(carla.TrafficLightState.Green)
        self._ego_traffic_light.set_green_time(self.timeout)

        self._other_traffic_light = CarlaDataProvider.get_next_traffic_light(self.other_actors[0], False)
        self._other_traffic_light.set_state(carla.TrafficLightState.Green)
        self._other_traffic_light.set_green_time(self.timeout)
        

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaActorPool.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        self.other_actors.append(first_vehicle)

    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")

    def _create_behavior(self):

        # EGO DRIVES
        ego_turn = 0 # drives straight
        target_waypoint = generate_target_waypoint(CarlaDataProvider.get_map().get_waypoint(self.ego_vehicles[0].get_location()), ego_turn)
        wp_choice = target_waypoint.next(30.0)
        target_waypoint = wp_choice[0]
        destination_location = target_waypoint.transform.location
        print(destination_location)
        ego_driving = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_driving.add_child(RssBasicAgentBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, destination_location))
        ego_driving.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], destination_location, 15))

        # POV DRIVES
        plan=[]
        turn = -1 # turn left
        plan, target_waypoint = generate_target_waypoint_list(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[0].get_location()), turn)

        # Generating waypoint list till next intersection
        wp_choice = target_waypoint.next(5.0)
        while len(wp_choice) == 1:
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(5.0)

        pov_driving = py_trees.composites.Sequence("Sequence Behavior")
        pov_driving_intersection = py_trees.composites.Parallel("ContinueDriving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        pov_driving_intersection.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_speed, plan=plan, avoid_collision=False))
        pov_driving_intersection.add_child(DriveDistance(self.other_actors[0], 40, name="Distance"))
        pov_driving.add_child(pov_driving_intersection)
        pov_driving.add_child(TimeOut(float('inf')))
        ########################
        parallel_drive = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(pov_driving)
        parallel_drive.add_child(ego_driving)
        # FINAL SEQUENCE
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(parallel_drive)
        #sequence.add_child(TimeOut(100))
        return sequence

    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename, self._rss_params)
        criteria.append(rss_criterion)
        return criteria