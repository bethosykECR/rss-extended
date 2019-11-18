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
from srunner.tools.scenario_config_parser import ScenarioConfigurationParser
#
from tools import annealing
from tools import robustness

from scenario_runner_extension.rss_aux import defineRssParams
from scenario_runner_extension.rss_aux import RssParamsInit
from scenario_runner_extension.rss_config_parser import parse_rss_scenario_configuration

from scenarios.rss_ext_forward_incursion import RssExtForwardIncursion
from scenarios.rss_ext_side_incursion import RssExtSideIncursion
from scenarios.rss_ext_shorter_following_distance import RssExtShorterFollowingDistance
from scenarios.rss_ext_platooning import RssExtPlatooning
from scenarios.rss_ext_merge import RssExtMerge
from scenarios.rss_ext_shorter_following_distance_multiple_geometry import RssExtShorterFollowingDistanceMG
from scenarios.rss_ext_occlusion import RssExtOcclusion
from scenarios.rss_ext_temp_occlusion import RssExtTempOcclusion
from scenarios.rss_ext_pass_pedestrian import RssExtPassPedestrian
from scenarios.rss_ext_parking_lot import RssExtParkingLot

#RES_FOLDER = '../results-' + time.strftime("%d-%H-%M-%S")
#if not os.path.exists(RES_FOLDER):
#    os.makedirs(RES_FOLDER)
#TRAJ_FILENAME = os.path.join(RES_FOLDER, 'trajectory.csv')

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
    client_timeout = 60.0  # in seconds
    ego_drive_world = 20.0  # in seconds
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
        # TODO:  Print this to a file instead
        # Scenario name, start time, end time, stats
        print('Start time')
        print(self._start_time)
        print('End time')
        print(self.world.tick())

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

        self._start_time = CarlaDataProvider.get_world().tick()

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

    def load_world(self, town):
        """
        Load a new CARLA world and provide data to CarlaActorPool and CarlaDataProvider
        """
        self.world = self.client.load_world(town)

        # Wait for the world to be ready
        self.world.tick()

        CarlaActorPool.set_client(self.client)
        CarlaActorPool.set_world(self.world)
        CarlaDataProvider.set_world(self.world)

        return True

    def load_and_run_scenario(self, args, config, scenario):
        """
        Load and run the given scenario
        """

        # Load scenario and run it
        self.manager.load_scenario(scenario)
        self.manager.run_scenario()

        # Provide outputs if required
        # TODO:  Add new metrics
        # self.analyze_scenario(args, config)

        # Stop scenario and cleanup
        self.manager.stop_scenario()
        scenario.remove_all_actors()

        self.cleanup()


    def simulate(self, config, args, rss_params):
        #file = open(self.filename_traj, 'w')
        #file.close()
        result = False
        scenario_class = self.get_scenario_class_or_fail(config.type)

        while not result:
            try:
                #moved to init() to load world only once
                #self.load_world(args, config.town)
                self.manager = ScenarioManager(self.world, args.debug)
                #CarlaActorPool.set_world(self.world)
                self.prepare_ego_vehicles(config)
                self.prepare_camera(config)
                scenario = scenario_class(self.world, rss_params, self.filename_traj, self.ego_vehicles, config, args.randomize, args.debug, variant=ARGUMENTS.scenario)
                result = True
            except Exception as exception:
                print("The scenario cannot be loaded")
                traceback.print_exc()
                print(exception)
                self.cleanup()
                pass
        self.load_and_run_scenario(args, config, scenario)
        #rob = robustness.getRobustness(self.filename_traj)

        #return rob
        return 0

    def run(self, args):
        scenario_config_file = ScenarioConfigurationParser.find_scenario_config(args.scenario, args.configFile) # xml file

        ####################################
        print(args.configFile)
        print(scenario_config_file)
        print(args.scenario)
        scenario_configurations = parse_rss_scenario_configuration(scenario_config_file, args.scenario)
        config = scenario_configurations[0] # since we work only with one scenario!

	# Defaults except for ones we set
        # Lon accel max: 3.500
        # Lon brake max: 8.000
        # Lon brake min: 4.000
        # Lon brake min correct: 0.100
        # Lat accel max: 0.200
        # Lat brake min: 0.800
        # Lat fluct mar: 0.000
        # Response time: 0.200

        g = 9.8
        rss_params = {}
        rss_params['alpha_lon_accel_max'] = 0.5*g
        rss_params['alpha_lon_brake_max'] = 0.7*g
        rss_params['response_time'] = 0.2
        rss_params['alpha_lon_brake_min'] = 0.5*g
        print('RSS params: %s' % rss_params)

        # Load the world once
        self.load_world(config.town)

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 0.015
        self.world.apply_settings(settings)

        # Set the sun to be directly overhead
        weather = carla.WeatherParameters(sun_altitude_angle=90)
        self.world.set_weather(weather)

        return self.simulate(config, args, rss_params)

if __name__ == '__main__':

    DESCRIPTION = ("CARLA RSS exploration\n")
    PARSER = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=RawTextHelpFormatter)
    PARSER.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: localhost)')
    PARSER.add_argument('--port', default='2000', help='TCP port to listen to (default: 2000)')
    PARSER.add_argument('--debug', action="store_true", help='Run with debug output')
    PARSER.add_argument('--scenario', default='Rss_Ext_FI_a', help='Name of the scenario to run')
    #PARSER.add_argument('--output', action="store_true", help='Provide results on stdout')
    #PARSER.add_argument('--file', action="store_true", help='Write results into a txt file')
    #PARSER.add_argument('--junit', action="store_true", help='Write results into a junit file')
    #PARSER.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')
    #PARSER.add_argument('--waitForEgo', action="store_true", help='Connect the scenario to an existing ego vehicle')
    #PARSER.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    PARSER.add_argument('--randomize', action="store_true", help='Scenario parameters are randomized')
    ARGUMENTS = PARSER.parse_args()
    ARGUMENTS.reloadWorld = True
    #ARGUMENTS.filename_traj = TRAJ_FILENAME
    ARGUMENTS.filename_traj = ""
    ARGUMENTS.configFile = os.path.join(os.getcwd(), 'rss.xml') # do not change this line
    ###############################################################
    # CHOOSE THE SCENARIO:
    ###############################################################
    # 1. Rss_Ext_FI:  Forward incursion
    #    Other vehicle cuts in between ego vehicle and lead vehicle.
    #       Rss_Ext_FI_a:  RSS Classic max braking response
    #       Rss_Ext_FI_b:  RSS Extended gentle braking response
    #
    # 2. Rss_Ext_SI:  Side incursion
    #    Other vehicle intrudes partially upon lane of ego vehicle
    #       Rss_Ext_SI_a:  Small intrusion.  RSS Classic ego vehicle stops.
    #       Rss_Ext_SI_b:  Small intrusion.  RSS Extended ego vehicle continues in-lane.
    #       Rss_Ext_SI_c:  Large intrusion.  RSS Extended ego vehicle changes lanes.
    #
    # 3. Rss_Ext_SFD:  Shorter following distance
    #     By setting limits on its own acceleration and braking, ego chooses a
    #     shorter following distance.
    #       Rss_Ext_SFD_a:  RSS Classic following distance
    #       Rss_Ext_SFD_b:  RSS Extended shorter following distance
    #
    # 4. Rss_PLAT:  Platooning
    #     Leveraging vehicle-to-vehicle communication could allow a vehicle
    #     to obtain a short response time and to coordinate acceleration.
    #     Similar implementation to Shorter Following Distance.
    #       Rss_PLAT_a:  RSS Classic following distance
    #       Rss_PLAT_b:  RSS Extended shorter following distance
    #
    #  5. Rss_Ext_MERGE:  Merge
    #    By setting limits on its own acceleration and braking, ego can merge
    #    into a tighter gap.
    #      Rss_Ext_MERGE_a: Current RSS library does not warn on merge case and ego may
    #         collide with follower vehicles.
    #      Rss_Ext_MERGE_b: RSS Classic behavior prohibits too-short safe following distance
    #         with new leader and ego cannot complete merge.
    #      Rss_Ext_MERGE_c: RSS Extended allows for shorter safe following distance and the
    #         ego vehicle is able to complete the merge.
    #
    #  6. Rss_Ext_SFDMG:  Shorter following distance multiple geometry
    #     By setting limits on its own acceleration and braking, ego chooses a
    #     shorter following distance entering a roundabout.
    #      Rss_Ext_SFDMG_a: Current RSS library does not yet include roundabouts and ego
    #         may collide with lead vehicle.
    #      Rss_Ext_SFDMG_b: Ego vehicle enters roundabout at RSS Classic safe following
    #         distance.
    #      Rss_Ext_SFDMG_c: RSS Extended ego vehicle sets limits on its own acceleration
    #         and braking to achieve a shorter following distance.
    #
    #  7. Rss_Ext_OCCL:  Occlusion
    #     Ego vehicle must decide yield behavior with respect to a vehicle
    #     traveling straight through a T intersection.  A tree partially
    #     occludes the intersection.
    #      Rss_Ext_OCCL_a: Current RSS library does not yet include T intersection checking
    #         and ego may collide with through vehicle.
    #      Rss_Ext_OCCL_b: RSS Classic assumes an incoming vehicle with the worst-case
    #         velocity (section 3.9), thus increasing the effect of the
    #         occlusion.
    #      Rss_Ext_OCCL_c: RSS Extended could permit an assumption of a lower velocity than
    #         worst-case.
    #
    #  8. Rss_Ext_TEMPOCCL:  Temporary occlusion of traffic signal
    #     The view of a traffic light is obstructed by a tall lead vehicle.
    #     The ego vehicle decides whether to proceed without visibility of the
    #     traffic light, or to wait until lead vehicle moves and the traffic
    #     light is visible.
    #     Rss_Ext_TEMPOCCL_a:  Ego vehicle proceeds without clear line of sight
    #     Rss_Ext_TEMPOCCL_b:  Ego vehicle waits for clear line of sight
    #
    #  9. Rss_Ext_PASSPED:  Passing a pedestrian
    #     The ego vehicle chooses a behavior when a pedestrian crosses the
    #     street.
    #     Rss_Ext_PASSPED_a:  Current RSS library does not cover pedestrians and the ego
    #         vehicle may collide with a pedestrian.  This corresponds to
    #         section 3.8 example 2 where the vehicle has the right-of-way.
    #     Rss_Ext_PASSPED_b:  The ego vehicle is required to yield to a pedestrian in a
    #         residential setting, section 3.8 example 1.
    #     Rss_Ext_PASSPED_c:  The ego vehicle is required to remain outisde of a pedestrian's
    #         expected route when walking along a residential road, section
    #         3.8 example 3.
    #
    # 10. Rss_Ext_PARKLOT:  Parking lot
    #     The ego vehicle drives through and exits a parking lot.
    #     Rss_Ext_PARKLOT_a:  The ego vehicle exits the lot.  Parking lots might not be
    #         considered driveable space by the RSS library yet, as the
    #         vehicle tries to exit immediately.  Also, the fence locations
    #         are not yet known by the ego vehicle.
    ###############################################################
    # Scenario name choices
    #ARGUMENTS.scenario = 'Rss_Ext_FI_a'
    #ARGUMENTS.scenario = 'Rss_Ext_FI_b'
    #ARGUMENTS.scenario = 'Rss_Ext_SI_a'
    #ARGUMENTS.scenario = 'Rss_Ext_SI_b'
    #ARGUMENTS.scenario = 'Rss_Ext_SI_c'
    #ARGUMENTS.scenario = 'Rss_Ext_SFD_a'
    #ARGUMENTS.scenario = 'Rss_Ext_SFD_b'
    #ARGUMENTS.scenario = 'Rss_Ext_PLAT_a'
    #ARGUMENTS.scenario = 'Rss_Ext_PLAT_b'
    #ARGUMENTS.scenario = 'Rss_Ext_MERGE_a'
    #ARGUMENTS.scenario = 'Rss_Ext_MERGE_b'
    #ARGUMENTS.scenario = 'Rss_Ext_MERGE_c'
    #ARGUMENTS.scenario = 'Rss_Ext_SFDMG_a'
    #ARGUMENTS.scenario = 'Rss_Ext_SFDMG_b'
    #ARGUMENTS.scenario = 'Rss_Ext_SFDMG_c'
    #ARGUMENTS.scenario = 'Rss_Ext_OCCL_a'
    #ARGUMENTS.scenario = 'Rss_Ext_OCCL_b'
    #ARGUMENTS.scenario = 'Rss_Ext_OCCL_c'
    #ARGUMENTS.scenario = 'Rss_Ext_TEMPOCCL_a'
    #ARGUMENTS.scenario = 'Rss_Ext_TEMPOCCL_b'
    #ARGUMENTS.scenario = 'Rss_Ext_PASSPED_a'
    #ARGUMENTS.scenario = 'Rss_Ext_PASSPED_b'
    #ARGUMENTS.scenario = 'Rss_Ext_PASSPED_c'
    #ARGUMENTS.scenario = 'Rss_Ext_PARKLOT_a'
    ###############################################################

    SCENARIORUNNER = None
    try:
        SCENARIORUNNER = ScenarioRunner(ARGUMENTS)
        SCENARIORUNNER.run(ARGUMENTS)
    finally:
        if SCENARIORUNNER is not None:
            del SCENARIORUNNER
