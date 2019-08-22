import carla 
import xml.etree.ElementTree as ET
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration
from srunner.tools.scenario_config_parser import ScenarioConfigurationParser


class CameraConfiguration(object):
    def __init__(self, node):
        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))
        pitch = float(node.attrib.get('pitch', 0))

        location = carla.Location(x=pos_x, y=pos_y, z=pos_z)
        rotation = carla.Rotation(yaw=yaw, pitch=pitch)

        self.transform = carla.Transform(location, rotation)


class RssScenarioConfiguration(ScenarioConfiguration):
    camera = None


def parse_rss_scenario_configuration(scenario_config_file, scenario_name):

	new_config = RssScenarioConfiguration()
	scenario_configurations = ScenarioConfigurationParser.parse_scenario_configuration(scenario_config_file, scenario_name)
	config = scenario_configurations[0]
	new_config.name = config.name
	new_config.town = config.town 
	new_config.type = config.type
	new_config.other_actors = config.other_actors
	new_config.ego_vehicles = config.ego_vehicles
	new_config.trigger_points = config.trigger_points
	new_config.target = config.target
	new_config.route = config.route

	rss_scenario_configurations = []
	tree = ET.parse(scenario_config_file)
	for scenario in tree.iter("scenario"):
		name = scenario.attrib.get('name', None)
		if name == scenario_name:        
		        for node in scenario.iter("camera"):
		        	new_config.camera = CameraConfiguration(node)
		        rss_scenario_configurations.append(new_config)
		        break
	return rss_scenario_configurations