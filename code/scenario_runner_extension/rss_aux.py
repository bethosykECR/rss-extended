import carla
import weakref
import inspect

class RssParams:
    def __init__(self, x):
        try:
            assert(len(x) == 2)
            self.alpha_lon_accel_max = x[0]
            self.alpha_lon_brake_max = x[1]
            #self.alpha_lon_brake_min = x[2]
            #self.alpha_lon_brake_min_correct = x[3]
            #self.alpha_lat_accel_max = x[4]
            #self.alpha_lat_brake_min = x[5]
            #self.lateral_fluctuation_margin = x[6]
            #self.response_time = x[3]
        except AssertionError as exception:
            print('x length is not rssParam appropriate')
            print(exception)

    def __str__(self):
        return ('X = (%.3f, %.3f)' % (self.alpha_lon_accel_max, self.alpha_lon_brake_max)) 
                                                                        #self.alpha_lon_brake_min,
                                                                        #self.alpha_lon_brake_min_correct, 
                                                                        #self.alpha_lat_accel_max, 
                                                                        #self.alpha_lat_brake_min, 
                                                                        #self.lateral_fluctuation_margin, 
                                                                        #self.response_time))


# ==============================================================================
# -- RssSensor --------------------------------------------------------
# ==============================================================================
class RssSensor(object):
    def __init__(self, parent_actor, rss_params):
        self.sensor = None
        self._parent = parent_actor
        self.timestamp = None
        self.response_valid = False
        self.lon_response = None
        self.lat_response_right = None
        self.lat_response_left = None
        self.acceleration_restriction = None
        self.ego_velocity = None
        self.rss_params = rss_params
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.rss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.0, z=0.0)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.

        def check_rss_class(clazz):
            return inspect.isclass(clazz) and "RssSensor" in clazz.__name__

        if not inspect.getmembers(carla, check_rss_class):
            raise RuntimeError('CARLA PythonAPI not compiled in RSS variant, please "make PythonAPI.rss"')
        weak_self = weakref.ref(self)
        self.sensor.visualize_results = True
        self.sensor.listen(lambda event: RssSensor._on_rss_response(weak_self, event))
        
        def print_dynamics(rss_dynamics):
            print('************************')
            print('RSS DYANMICS:')
            print('Lon accel max: %.3f' % rss_dynamics.alpha_lon.accel_max.value)
            print('Lon break max: %.3f' % rss_dynamics.alpha_lon.brake_max.value)
            print('Lon break min: %.3f' % rss_dynamics.alpha_lon.brake_min.value)
            print('Lon break min correct: %.3f' % rss_dynamics.alpha_lon.brake_min_correct.value)
            #
            print('Lat accel max: %.3f' % rss_dynamics.alpha_lat.accel_max.value)
            print('Lat break min: %.3f' % rss_dynamics.alpha_lat.brake_min.value)
            #
            print('Lat fluct mar: %.3f' % rss_dynamics.lateral_fluctuation_margin.value)
            print('Response time: %.3f' % rss_dynamics.response_time.value)
            print('************************')

        def set_parameters(rss_dynamics, rss_params):
            rss_dynamics.alpha_lon.accel_max =         carla.Acceleration(rss_params.alpha_lon_accel_max)
            rss_dynamics.alpha_lon.brake_max =         carla.Acceleration(rss_params.alpha_lon_brake_max)
            rss_dynamics.alpha_lon.brake_min =         carla.Acceleration(8.0)
            #rss_dynamics.alpha_lon.brake_min_correct = carla.Acceleration(rss_params.alpha_lon_brake_min_correct)
            #rss_dynamics.alpha_lat.accel_max =         carla.Acceleration(rss_params.alpha_lat_accel_max)
            #rss_dynamics.alpha_lat.brake_min =         carla.Acceleration(rss_params.alpha_lat_brake_min)
            #rss_dynamics.lateral_fluctuation_margin =  carla.Distance(rss_params.lateral_fluctuation_margin)
            #rss_dynamics.response_time =               carla.Duration(rss_params.response_time)     
            rss_dynamics.response_time =               carla.Duration(0.1)     
            return rss_dynamics

        rss_dynamics = set_parameters(self.sensor.ego_vehicle_dynamics, self.rss_params)
        self.sensor.ego_vehicle_dynamics = rss_dynamics
        print_dynamics(self.sensor.ego_vehicle_dynamics)

    @staticmethod
    def _on_rss_response(weak_self, response):
        self = weak_self()
        if not self:
            return
        self.timestamp = response.timestamp
        self.response_valid = response.response_valid
        self.lon_response = response.longitudinal_response
        self.lat_response_right = response.lateral_response_right
        self.lat_response_left = response.lateral_response_left
        self.acceleration_restriction = response.acceleration_restriction
        self.ego_velocity = response.ego_velocity