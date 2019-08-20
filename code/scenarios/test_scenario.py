import carla
import py_trees

from scenario_runner_extension.rss_behavior import RssBasicAgentBehavior
from scenario_runner_extension.rss_criteria import RssTest

from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from agents.navigation.local_planner import RoadOption
from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.tools.scenario_helper import get_waypoint_in_distance

class RssTestScenario(FollowLeadingVehicle):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        
        self._rss_params = rss_params
        self._filename = filename
        self.timeout=100 # seconds
        super(RssTestScenario, self).__init__(world, 
                                                     ego_vehicles, 
                                                     config,
                                                     randomize,
                                                     debug_mode,
                                                     criteria_enable,
                                                     self.timeout)

    def _initialize_actors(self, config):

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        self.other_actors.append(first_vehicle)

    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")
         
    def _create_behavior(self):

        target_v_kmh = self._first_vehicle_speed
        target_v_mps = target_v_kmh / 3.6
        #
        acceleration_value = 0.7 # how to turn into g?
        braking_value_soft = 0.1 #
        braking_value_hard = 0.6 #
        wait_time = 6 #sec
        intersection_location = carla.Location(-2, 315, 0.5) 
        dist_endcond = 60 # depends on the behavior of the front car. change if the acc/braking vals are different
        ##############################################
        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        #------------------------------------
        plan = []
        wp = CarlaDataProvider.get_map().get_waypoint(intersection_location)
        plan.append((wp, RoadOption.LANEFOLLOW))

        p1 = WaypointFollower(self.other_actors[0], target_v_kmh, plan=plan, avoid_collision=True)
        #------------------------------------
        p8 = TimeOut(float('inf'))
        #------------------------------------
        pov_driving = py_trees.composites.Sequence()
        pov_driving.add_child(p1)
        pov_driving.add_child(p8)
        ##############################################

        endcondition_part1 = StandStill(self.ego_vehicles[0], name="StandStill")
        endcondition_part2 = InTriggerDistanceToLocation(self.ego_vehicles[0], intersection_location, dist_endcond)
        endcondition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        ego_driving = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_driving.add_child(RssBasicAgentBehavior(self._rss_params, self.ego_vehicles[0], intersection_location))
        ego_driving.add_child(endcondition)
        ##############################################

        parallel_drive = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(pov_driving)
        parallel_drive.add_child(ego_driving)

        seq = py_trees.composites.Sequence()
        seq.add_child(start_transform)
        seq.add_child(parallel_drive)
        seq.add_child(ActorDestroy(self.other_actors[0]))

        return seq


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename, self._rss_params)
        criteria.append(rss_criterion)
        return criteria