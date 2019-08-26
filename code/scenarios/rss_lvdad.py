import carla
import py_trees

from scenario_runner_extension.rss_behavior import RssBasicAgentBehavior
from scenario_runner_extension.rss_criteria import RssTest

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *

class RssLVDAD(BasicScenario):

    def __init__(self, world, rss_params, filename, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        
        self._rss_params = rss_params
        self._ego_target_speed = config.ego_vehicles[0].target_speed
        self._other_actor_target_speed = config.other_actors[0].target_speed
        self._target = config.target.transform.location
        self._filename = filename
        self.timeout=45 # seconds
        self._other_actor_transform = None

        super(RssLVDAD, self).__init__("LVDAD",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

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
       
        acceleration_value = 0.7 # how to turn into g?
        braking_value_soft = 0.1 #
        braking_value_hard = 0.6 #
        wait_time = 6 #sec
        dist_endcond = 60 # depends on the behavior of the front car. change if the acc/braking vals are different
        ##############################################
        #------------------------------------
        p1 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p1.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_speed))
        p1.add_child(TriggerVelocity(self.other_actors[0], self._other_actor_target_speed))
        #------------------------------------
        p2 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p2.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_speed))
        p2.add_child(TimeOut(wait_time))
        #------------------------------------
        p3 = StopVehicle(self.other_actors[0], braking_value_soft)
        #-------------------------
        p4 = TimeOut(wait_time)
        #------------------------------------
        p5 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p5.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_speed))
        p5.add_child(AccelerateToVelocity(self.other_actors[0], acceleration_value, self._other_actor_target_speed))
        #------------------------------------
        p6 = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        p6.add_child(WaypointFollower(self.other_actors[0], self._other_actor_target_speed))
        p6.add_child(TimeOut(wait_time))
        #------------------------------------
        p7 = StopVehicle(self.other_actors[0], braking_value_hard)
        #------------------------------------
        p8 = TimeOut(float('inf'))
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
        endcondition_part2 = InTriggerDistanceToLocation(self.ego_vehicles[0], self._target, dist_endcond)
        endcondition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        ego_driving = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_driving.add_child(RssBasicAgentBehavior(self._rss_params, self.ego_vehicles[0], self._ego_target_speed, self._target))
        ego_driving.add_child(endcondition)
        ##############################################

        parallel_drive = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(pov_driving)
        parallel_drive.add_child(ego_driving)

        seq = py_trees.composites.Sequence()
        seq.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        seq.add_child(parallel_drive)
        seq.add_child(ActorDestroy(self.other_actors[0]))
        seq.add_child(ActorDestroy(self.ego_vehicles[0]))

        return seq


    def _create_test_criteria(self):
        criteria = []
        rss_criterion = RssTest(self.ego_vehicles[0], self._filename)
        criteria.append(rss_criterion)
        return criteria