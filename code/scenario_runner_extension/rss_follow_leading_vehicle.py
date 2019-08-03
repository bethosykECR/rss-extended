import carla
import py_trees

from scenario_runner_extension.rss_behavior import RssBasicAgentBehavior
from scenario_runner_extension.rss_behavior import WaitForSeconds
from scenario_runner_extension.rss_criteria import RssTest

from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.tools.scenario_helper import get_waypoint_in_distance


class RssFollowLeadingVehicle(FollowLeadingVehicle):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        
        self._rss_params = rss_params
        self._filename = filename
        self.timeout=30
        super(RssFollowLeadingVehicle, self).__init__(world, 
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
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # to avoid the other actor blocking traffic, it was spawed elsewhere
        # reset its pose to the required one
        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        other_driving_to_next_intersection = py_trees.composites.Parallel("DrivingTowardsIntersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        other_driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        other_driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))

        other_stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)
      
        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill")
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        intersection_location = carla.Location(326, 133, 0.7)   
        ego_drives = py_trees.composites.Parallel("ego", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_drives.add_child(RssBasicAgentBehavior(self._rss_params, self.ego_vehicles[0], intersection_location))
        ego_drives.add_child(endcondition)


        other_drives = py_trees.composites.Sequence("Sequence Behavior")
        other_drives.add_child(other_driving_to_next_intersection)
        other_drives.add_child(other_stop)
        other_drives.add_child(endcondition)
       
        parallel_drive = py_trees.composites.Parallel("DrivingTowardsIntersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        parallel_drive.add_child(other_drives)
        parallel_drive.add_child(ego_drives)


        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(WaitForSeconds(3))
        sequence.add_child(parallel_drive)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename)
        criteria.append(rss_criterion)
        return criteria


class RssLVDAD(FollowLeadingVehicle):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        
        self._rss_params = rss_params
        self._filename = filename
        self.timeout=100 # seconds
        super(RssLVDAD, self).__init__(world, 
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
        p1 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p1.add_child(WaypointFollower(self.other_actors[0], target_v_kmh))
        p1.add_child(TriggerVelocity(self.other_actors[0], target_v_mps))
        #------------------------------------
        p2 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p2.add_child(WaypointFollower(self.other_actors[0], target_v_kmh))
        p2.add_child(WaitForSeconds(wait_time))
        #------------------------------------
        p3 = StopVehicle(self.other_actors[0], braking_value_soft)
        #-------------------------
        p4 = WaitForSeconds(wait_time)
        #------------------------------------
        p5 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p5.add_child(WaypointFollower(self.other_actors[0], target_v_kmh))
        p5.add_child(AccelerateToVelocity(self.other_actors[0], acceleration_value, target_v_mps))
        #------------------------------------
        p6 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p6.add_child(WaypointFollower(self.other_actors[0], target_v_kmh))
        p6.add_child(WaitForSeconds(wait_time))
        #------------------------------------
        p7 = StopVehicle(self.other_actors[0], braking_value_hard)
        #------------------------------------
        p8 = WaitForSeconds(float('inf'))
        #------------------------------------
        pov_driving = py_trees.composites.Sequence()
        pov_driving.add_child(p1)
        pov_driving.add_child(p2)
        pov_driving.add_child(p3)
        pov_driving.add_child(p4)
        pov_driving.add_child(p5)
        pov_driving.add_child(p6)
        pov_driving.add_child(p7)
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
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename)
        criteria.append(rss_criterion)
        return criteria