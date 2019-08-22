from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
import carla
import sys
import os
import time

sys.path.append(os.getenv('ROOT_SCENARIO_RUNNER'))
from srunner.scenariomanager.carla_data_provider import CarlaActorPool, CarlaDataProvider
from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.tools.config_parser import find_scenario_config
#
from tools import annealing
from tools import robustness

from scenario_runner_extension.rss_aux import defineRssParams
from scenario_runner_extension.rss_aux import RssParamsInit
from scenario_runner_extension.rss_config_parser import parse_rss_scenario_configuration 

from scenarios.rss_opposite_vehicle_taking_priority import RssOppositeVehicleRunningRedLight
from scenarios.rss_lvdad import RssLVDAD
from scenarios.rss_follow_leading_vehicle import RssFollowLeadingVehicle
from scenarios.rss_pov_unprotected_left import RssPovUnprotectedLeft

RES_FOLDER = '../results-' + time.strftime("%d-%H-%M-%S")
if not os.path.exists(RES_FOLDER):
    os.makedirs(RES_FOLDER)
TRAJ_FILENAME = os.path.join(RES_FOLDER, 'trajectory.csv')



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
    world = None
    manager = None
    additional_scenario_module = None

    def __init__(self, args):
        self.filename_traj = args.filename_traj
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
        #settings.synchronous_mode = True
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

    def get_scenario_class_or_fail(self, scenario):
        """
        Get scenario class by scenario name
        If scenario is not supported or not found, exit script
        """
        if scenario in globals():
            return globals()[scenario]
        print("Scenario '{}' not supported ... Exiting".format(scenario))
        sys.exit(-1)

    def cleanup(self, ego=False):
        """
        Remove and destroy all actors
        """
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

        CarlaDataProvider.cleanup()
        CarlaActorPool.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if ego:
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

    def prepare_camera(self, config):
        spectator = self.world.get_spectator()
        spectator.set_transform(config.camera.transform)


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


    def simulate(self, config, args, rss_params):
        file = open(self.filename_traj, 'wb')  
        file.close()
        result = False
        scenario_class = self.get_scenario_class_or_fail(config.type)

        while not result:
            try:
                self.load_world(args, config.town)
                self.manager = ScenarioManager(self.world, args.debug)   
                CarlaActorPool.set_world(self.world)
                self.prepare_ego_vehicles(config)
                self.prepare_camera(config)    
                scenario = scenario_class(self.world, rss_params, self.filename_traj, self.ego_vehicles, config, args.randomize, args.debug)
                result = True
            except Exception as exception:
                print("The scenario cannot be loaded")
                traceback.print_exc()
                print(exception)
                self.cleanup()
                pass
        self.load_and_run_scenario(args, config, scenario)
        rob = robustness.getRobustness(self.filename_traj)

        return rob


    def run(self, args):
        scenario_config_file = find_scenario_config(args.scenario, args.configFile) # xml file 

        num_simult_runs = 1
        nruns = 5000
        ####################################

        search_names = ['alpha_lon_accel_max', 'response_time']
        alpha_lon_accel_max = 3.5
        response_time = 1.0
        ####################################
        x0, searchSpace = RssParamsInit().getInit(search_names,
                                                   alpha_lon_accel_max = alpha_lon_accel_max,
                                                   response_time = response_time)
        print('X0 = %s' % x0)
        print('SearchSpace = %s\n' % searchSpace)
        #-------------------------------
        X0 =[]
        for _ in xrange(num_simult_runs):
            X0.append(x0)
        ####################################


        ####################################        
        def ff(x):
            scenario_configurations = parse_rss_scenario_configuration(scenario_config_file, args.scenario)
            config = scenario_configurations[0] # since we work only with one scenario!

            rss_params = defineRssParams(x, search_names)
            rss_params['alpha_lon_brake_min_correct'] = 0.1
            print('RSS params: %s' % rss_params)

            return self.simulate(config, args, rss_params)

        # reproduce trajs
        '''
        for i in range(10):
            self.filename_traj = os.path.join(RES_FOLDER, ('trajectory'+str(i)+'.csv'))
            ff(x0)
        '''
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
    ARGUMENTS.configFile = os.path.join(os.getcwd(), 'rss.xml') # do not change this line
    # but do change this scenario:
    #ARGUMENTS.scenario = 'Rss_FollowLeadingVehicle_1'
    ARGUMENTS.scenario = 'Rss_lvdad'
    #ARGUMENTS.scenario = 'Rss_OppositeVehicleRunningRedLight011'
    #ARGUMENTS.scenario = 'Rss_PovUnprotectedLeft'

    
    ARGUMENTS.reloadWorld = True

    SCENARIORUNNER = None
    try:
        SCENARIORUNNER = ScenarioRunner(ARGUMENTS)
        SCENARIORUNNER.run(ARGUMENTS)
    finally:
        if SCENARIORUNNER is not None:
            del SCENARIORUNNER
