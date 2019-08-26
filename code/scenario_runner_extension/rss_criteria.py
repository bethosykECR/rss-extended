import py_trees
from tools import dist_aux
from tools import other_aux
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion
import math

class RssTest(Criterion):

    def __init__(self, actor, filename, optional=False, name="CheckRss", terminate_on_failure=False):
        super(RssTest, self).__init__(name, actor, 0, None, optional, terminate_on_failure)
        world = self.actor.get_world()
        self.vehicles = world.get_actors().filter('vehicle.*')
        self.filename = filename
        self.actor = actor


    def update(self):
        # get distance between BBX
        d = dist_aux.evaluate_dist(self.vehicles)

        # get ego velocity 
        ego_v = self.actor.get_velocity()
        ego_velocity = math.sqrt(ego_v.x**2 + ego_v.y**2 + ego_v.z**2) # [m/s]
          
        pov = [actor for actor in self.vehicles if actor.attributes.get('role_name') != 'hero'][0]
        pov_v = pov.get_velocity()
        pov_velocity = math.sqrt(pov_v.x**2 + pov_v.y**2 + pov_v.z**2) # [m/s]
        #print('EGO_V = %s, POV_V = %s' % (ego_velocity, pov_velocity))
        #pos_ego = self.actor.get_location()
        #pos_pov = pov.get_location()
        #print('pos_ego = (%.6f, %.6f, %.6f), pos_pov = (%.6f, %.6f, %.6f)' % (pos_ego.x, pos_ego.y, pos_ego.z, pos_pov.x, pos_pov.y, pos_pov.z)) 
        other_aux.write2csv(self.filename, [d, ego_velocity, pov_velocity])
        return py_trees.common.Status.RUNNING


        #self.rss_params = rss_params
        #self.rss_dynamics = RssSensor(self.actor, self.rss_params).sensor.ego_vehicle_dynamics
        '''
        d_rss = max(0, 
                ego_velocity * self.rss_dynamics.response_time.value + 
                1/2*self.rss_dynamics.alpha_lon.accel_max.value * self.rss_dynamics.response_time.value** +
                1/(2*self.rss_dynamics.alpha_lon.brake_min.value)*
                    (ego_velocity + self.rss_dynamics.alpha_lon.accel_max.value*self.rss_dynamics.response_time.value)** -
                1/(2*self.rss_dynamics.alpha_lon.brake_max.value)* pov_velocity)
        '''