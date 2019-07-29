from __future__ import print_function



import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
import importlib
import inspect

import carla
import numpy as np
import sys

from tools import dist_aux
from tools import other_aux
from tools import annealing
from tools import robustness

sys.path.append('/home/user/alena/scenario_runner')
from scenario_runner_extension.rss_aux import RssParams
from scenario_runner_extension.rss_follow_leading_vehicle import RssFollowLeadingVehicle

import os 
RES_FOLDER = '/home/user/alena/results'
if not os.path.exists(RES_FOLDER):
    os.makedirs(RES_FOLDER)
import time
TRAJ_FILENAME = os.path.join(RES_FOLDER, 'trajectory.csv')
ROB_FILENAME = os.path.join(RES_FOLDER, ('rob-'+time.strftime("%d-%H-%M-%S")+'.csv'))


from srunner.scenariomanager.carla_data_provider import *
from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.scenarios.background_activity import *
from srunner.scenarios.control_loss import *
from srunner.scenarios.follow_leading_vehicle import *
from srunner.scenarios.maneuver_opposite_direction import *
from srunner.scenarios.master_scenario import *
from srunner.scenarios.no_signal_junction_crossing import *
from srunner.scenarios.object_crash_intersection import *
from srunner.scenarios.object_crash_vehicle import *
from srunner.scenarios.opposite_vehicle_taking_priority import *
from srunner.scenarios.other_leading_vehicle import *
from srunner.scenarios.signalized_junction_left_turn import *
from srunner.scenarios.signalized_junction_right_turn import *
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.config_parser import *

from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle



class ScenarioRunner(object):

    """
    This is the core scenario runner module. It is responsible for
    running (and repeating) a single scenario or a list of scenarios.

    Usage:
    scenario_runner = ScenarioRunner(args)
    scenario_runner.run(args)
    del scenario_runner
    """

    ego_vehicles = []

    # Tunable parameters
    client_timeout = 30.0  # in seconds
    wait_for_world = 20.0  # in seconds
    frame_rate = 20.0      # in Hz
    world = None
    manager = None
    additional_scenario_module = None

    def __init__(self, args):
        self.filename_traj = args.filename_traj
        self.filename_rob = args.filename_rob
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = carla.Client(args.host, int(args.port))
        self.client.set_timeout(self.client_timeout)

        # Once we have a client we can retrieve the world that is currently
        # running.
        self.world = self.client.get_world()

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 1.0 / self.frame_rate
        self.world.apply_settings(settings)

        CarlaActorPool.set_world(self.world)
        CarlaDataProvider.set_world(self.world)


    def __del__(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """
        self.cleanup(True)
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world

    def cleanup(self, ego=False):
        """
        Remove and destroy all actors
        """
        CarlaDataProvider.cleanup()
        CarlaActorPool.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if ego:
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

    def prepare_ego_vehicles(self, config, wait_for_ego_vehicles=False):
        """
        Spawn or update the ego vehicle according to
        its parameters provided in config

        As the world is re-loaded for every scenario, no ego exists so far
        """

        if not wait_for_ego_vehicles:
            for vehicle in config.ego_vehicles:
                self.ego_vehicles.append(CarlaActorPool.setup_actor(vehicle.model,
                                                                    vehicle.transform,
                                                                    vehicle.rolename,
                                                                    True))
        else:
            ego_vehicle_missing = True
            while ego_vehicle_missing:
                self.ego_vehicles = []
                ego_vehicle_missing = False
                for ego_vehicle in config.ego_vehicles:
                    ego_vehicle_found = False
                    carla_vehicles = CarlaDataProvider.get_world().get_actors().filter('vehicle.*')
                    for carla_vehicle in carla_vehicles:
                        if carla_vehicle.attributes['role_name'] == ego_vehicle.rolename:
                            ego_vehicle_found = True
                            self.ego_vehicles.append(carla_vehicle)
                            break
                    if not ego_vehicle_found:
                        ego_vehicle_missing = True
                        break

            for i, _ in enumerate(self.ego_vehicles):
                self.ego_vehicles[i].set_transform(config.ego_vehicles[i].transform)

        # sync state
        CarlaDataProvider.get_world().tick()

    def analyze_scenario(self, args, config):
        """
        Provide feedback about success/failure of a scenario
        """

        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        config_name = config.name
        if args.outputDir != '':
            config_name = os.path.join(args.outputDir, config_name)
        if args.junit:
            junit_filename = config_name + current_time + ".xml"
        filename = None
        if args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(args.output, filename, junit_filename):
            print("Success!")
        else:
            print("Failure!")

    def load_world(self, args, town):
        """
        Load a new CARLA world and provide data to CarlaActorPool and CarlaDataProvider
        """

        if args.reloadWorld:
            self.world = self.client.load_world(town)
        else:
            if CarlaDataProvider.get_map().name != town:
                print("The CARLA server uses the wrong map!")
                print("This scenario requires to use map {}".format(town))
                return False

        CarlaActorPool.set_client(self.client)
        CarlaActorPool.set_world(self.world)
        CarlaDataProvider.set_world(self.world)

        # Wait for the world to be ready
        self.world.tick()
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 1.0 / self.frame_rate
        self.world.apply_settings(settings)

        return True

    def load_and_run_scenario(self, args, config, scenario):
        """
        Load and run the given scenario
        """

        # Load scenario and run it
        self.manager.load_scenario(scenario)
        self.manager.run_scenario()

        # Provide outputs if required
        self.analyze_scenario(args, config)

        # Stop scenario and cleanup
        self.manager.stop_scenario()
        scenario.remove_all_actors()

        self.cleanup()


    def simulate(self, config, args, x):
        rss_params = RssParams(x)
        print(rss_params)  
        file = open(args.filename_traj, 'wb')  
        file.close()

        result = False
        while not result:
            try:
                self.load_world(args, config.town)
                self.manager = ScenarioManager(self.world, args.debug)   
                CarlaActorPool.set_world(self.world)
                self.prepare_ego_vehicles(config)
                scenario = RssFollowLeadingVehicle(self.world, rss_params, self.filename_traj, self.ego_vehicles, config, args.randomize, args.debug)
                result = True
            except Exception as exception:
                print("The scenario cannot be loaded")
                traceback.print_exc()
                print(exception)
                self.cleanup()
                pass
        self.load_and_run_scenario(args, config, scenario)
        rob = robustness.getRobustness(args.filename_traj)

        other_aux.write2csv(args.filename_rob, rob)
        return rob



    def run(self, args):
        scenario_config_file = find_scenario_config(args.scenario, args.configFile) # xml file
        scenario_configurations = parse_scenario_configuration(scenario_config_file, args.scenario)
        config = scenario_configurations[0] # since we work only with one scenario!

        num_simult_runs = 1
        nruns = 200
        ####################################
        # 0 
        alpha_lon_accel_max = 16.62885703232409
        alpha_lon_accel_max_min = 0.0
        alpha_lon_accel_max_max = 20.0
        # 1
        alpha_lon_break_max = 64.4225632047746
        alpha_lon_break_max_min = 8
        alpha_lon_break_max_max = 100.0
        # 2
        #alpha_lon_brake_min = 4.0
        #alpha_lon_brake_min = 100.0
        #alpha_lon_brake_min_min = 0.0
        #alpha_lon_brake_min_max = 6.0
        
        # 3
        #alpha_lon_brake_min_correct = 3.0
        #alpha_lon_brake_min_correct_min = 0.0
        #alpha_lon_brake_min_correct_max = 3.5
        # 4
        #alpha_lat_accel_max = 0.2
        #alpha_lat_accel_max_min = 0.0
        #alpha_lat_accel_max_max = 2
        # 5
        #alpha_lat_brake_min = 0.8
        #alpha_lat_brake_min_min = 0.0
        #alpha_lat_brake_min_max = 2
        # 6
        #lateral_fluctuation_margin = 0.0
        #lateral_fluctuation_margin_min = 0.0
        #lateral_fluctuation_margin_max = 0.01
        # 7
        #response_time = 0.01
        #response_time_min = 0.01
        #response_time_max = 10.0

        #################################
        #true:
        #alpha_lon_accel_max = 3.5
        #alpha_lon_break_max = 8.0
        #alpha_lon_brake_min = 4.0
        #alpha_lon_brake_min_correct = 3.0
        #alpha_lat_accel_max = 0.2
        #alpha_lat_brake_min = 0.8
        #lateral_fluctuation_margin = 0.0
        #response_time = 1.0
        ####################################
        x0 = np.array([alpha_lon_accel_max,
                       alpha_lon_break_max])
                       #alpha_lon_brake_min,
                       #alpha_lon_brake_min_correct,
                       #alpha_lat_accel_max,
                       #alpha_lat_brake_min,
                       #lateral_fluctuation_margin,
                       #response_time])
        X0 =[]
        for _ in xrange(num_simult_runs):
            X0.append(x0)

        ####################################
        searchSpace = np.array([[alpha_lon_accel_max_min, alpha_lon_accel_max_max], 
                                [alpha_lon_break_max_min, alpha_lon_break_max_max]])
                                #[alpha_lon_brake_min_min, alpha_lon_brake_min_max],
                                #[alpha_lon_brake_min_correct_min, alpha_lon_brake_min_correct_max],
                                #[alpha_lat_accel_max_min, alpha_lat_accel_max_max],
                                #[alpha_lat_brake_min_min, alpha_lat_brake_min_max],
                                #[lateral_fluctuation_margin_min, lateral_fluctuation_margin_max],
                                #[response_time_min, response_time_max]])

        def ff(x):
            return self.simulate(config, args, x)

        best_x_history, best_f_history, x_history, f_history, accept_x_history, accept_flags = annealing.runFunc(ff, X0, searchSpace, nruns, num_simult_runs, RES_FOLDER) 




if __name__ == '__main__':

    DESCRIPTION = ("CARLA RSS exploration\n")
    PARSER = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=RawTextHelpFormatter)
    PARSER.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: localhost)')
    PARSER.add_argument('--port', default='2000', help='TCP port to listen to (default: 2000)')
    PARSER.add_argument('--debug', action="store_true", help='Run with debug output')
    PARSER.add_argument('--output', action="store_true", help='Provide results on stdout')
    PARSER.add_argument('--file', action="store_true", help='Write results into a txt file')
    PARSER.add_argument('--junit', action="store_true", help='Write results into a junit file')
    PARSER.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')
    PARSER.add_argument('--waitForEgo', action="store_true", help='Connect the scenario to an existing ego vehicle')
    PARSER.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    PARSER.add_argument('--randomize', action="store_true", help='Scenario parameters are randomized')
    ARGUMENTS = PARSER.parse_args()

    ARGUMENTS.filename_traj = TRAJ_FILENAME
    ARGUMENTS.filename_rob = ROB_FILENAME
    ARGUMENTS.scenario = 'FollowLeadingVehicle_1'
    ARGUMENTS.reloadWorld = True

    SCENARIORUNNER = None
    try:
        SCENARIORUNNER = ScenarioRunner(ARGUMENTS)
        SCENARIORUNNER.run(ARGUMENTS)
    finally:
        if SCENARIORUNNER is not None:
            del SCENARIORUNNER
