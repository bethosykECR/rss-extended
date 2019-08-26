import carla
import py_trees

from scenario_runner_extension.rss_behavior import RssBasicAgentBehavior
from scenario_runner_extension.rss_criteria import RssTest

from srunner.scenarios.opposite_vehicle_taking_priority import OppositeVehicleRunningRedLight
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *
from srunner.tools.scenario_helper import get_waypoint_in_distance
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *


class RssOppositeVehicleRunningRedLight(OppositeVehicleRunningRedLight):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        
        self._rss_params = rss_params
        self._filename = filename
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self.timeout=30
        super(RssOppositeVehicleRunningRedLight, self).__init__(world, 
                                                     ego_vehicles, 
                                                     config,
                                                     randomize,
                                                     debug_mode,
                                                     criteria_enable,
                                                     self.timeout)
        self._other_actor_target_velocity = config.other_actors[0].target_speed
        # attr self._other_actor_target_velocity is defined in parent OppositeVehicleRunningRedLight

    def _setup_scenario_trigger(self, config):
        return StandStill(self.ego_vehicles[0], name="StandStill")
         
    def _create_behavior(self):
     
        crossing_point_dynamic = get_crossing_point(self.ego_vehicles[0])

        # start condition
        startcondition = InTriggerDistanceToLocation(self.ego_vehicles[0], crossing_point_dynamic, self._ego_distance_to_traffic_light, name="Waiting for start position")

        sync_arrival_parallel = py_trees.composites.Parallel("Synchronize arrival times", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        location_of_collision_dynamic = get_geometric_linear_intersection(self.ego_vehicles[0], self.other_actors[0])
        sync_arrival = SyncArrival(self.other_actors[0], self.ego_vehicles[0], location_of_collision_dynamic)
        sync_arrival_stop = InTriggerDistanceToNextIntersection(self.other_actors[0], 5)
        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(sync_arrival_stop)

        # Generate plan for WaypointFollower
        turn = 0  # drive straight ahead
        plan = []
        # generating waypoints until intersection (target_waypoint)

        target_waypoint = generate_target_waypoint(CarlaDataProvider.get_map().get_waypoint(self.other_actors[0].get_location()), turn)
        #print(target_waypoint)
        #print('------------')
        # Generating waypoint list till next intersection
        wp_choice = target_waypoint.next(5.0)
        while len(wp_choice) == 1:
            target_waypoint = wp_choice[0]
            #print(target_waypoint)
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(5.0)
        #print(wp_choice[0])
        #print(wp_choice[1])

        continue_driving = py_trees.composites.Parallel("ContinueDriving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        continue_driving.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_velocity, plan=plan, avoid_collision=False))
        continue_driving.add_child(DriveDistance(self.other_actors[0], self._other_actor_distance, name="Distance"))
        continue_driving.add_child(TimeOut(10))

        # finally wait that ego vehicle drove a specific distance
        #wait = DriveDistance(self.ego_vehicles[0], self._ego_distance_to_drive, name="DriveDistance")

        pov_driving = py_trees.composites.Sequence()
        pov_driving.add_child(startcondition)
        pov_driving.add_child(sync_arrival_parallel)
        pov_driving.add_child(continue_driving)
        #pov_driving.add_child(wait)
        pov_driving.add_child(TimeOut(float('inf')))

        ego_turn = -1 # turn left
        _, target_waypoint = generate_target_waypoint_list(CarlaDataProvider.get_map().get_waypoint(self.ego_vehicles[0].get_location()), ego_turn)

        # Generating waypoint list till next intersection
        wp_choice = target_waypoint.next(30.0)
        target_waypoint = wp_choice[0]
            

        destination_location = target_waypoint.transform.location
        #print(destination_location)
        ego_driving = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_driving.add_child(RssBasicAgentBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, destination_location))
        ego_driving.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], destination_location, 15))

        # Build behavior tree
        parallel_drive = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(pov_driving)
        parallel_drive.add_child(ego_driving)

        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(parallel_drive)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename, self._rss_params)
        criteria.append(rss_criterion)
        return criteria
