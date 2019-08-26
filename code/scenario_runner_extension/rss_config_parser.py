import carla 
import xml.etree.ElementTree as ET
from srunner.scenarioconfigs.scenario_configuration import ActorConfiguration
from srunner.scenarioconfigs.route_scenario_configuration import TargetConfiguration


class RssActorConfiguration(ActorConfiguration):
    def __init__(self, node, rolename):
    	self.target_speed = float(node.attrib.get('target_speed', 0)) 

    	super(RssActorConfiguration, self).__init__(node, rolename)


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


class RssScenarioConfiguration(object):
    ego_vehicles = []
    other_actors = []
    town = None
    name = None
    type = None
    target = None
    camera = None


def parse_rss_scenario_configuration(scenario_config_file, scenario_name):

    tree = ET.parse(scenario_config_file)

    rss_scenario_configurations = []

    for scenario in tree.iter("scenario"):

        new_config = RssScenarioConfiguration()
        new_config.town = scenario.attrib.get('town', None)
        new_config.name = scenario.attrib.get('name', None)
        new_config.type = scenario.attrib.get('type', None)
        new_config.other_actors = []
        new_config.ego_vehicles = []

        for ego_vehicle in scenario.iter("ego_vehicle"):
            new_config.ego_vehicles.append(RssActorConfiguration(ego_vehicle, 'hero'))
            #new_config.trigger_points.append(new_config.ego_vehicles[-1].transform)

        for target in scenario.iter("target"):
            new_config.target = TargetConfiguration(target)

        for other_actor in scenario.iter("other_actor"):
            new_config.other_actors.append(RssActorConfiguration(other_actor, 'scenario'))

        for node in scenario.iter("camera"):
        	new_config.camera = CameraConfiguration(node)

        if new_config.name == scenario_name:
        	rss_scenario_configurations.append(new_config)

    return rss_scenario_configurations

