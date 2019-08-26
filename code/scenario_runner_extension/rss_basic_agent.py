import carla
from agents.navigation.basic_agent import BasicAgent
from scenario_runner_extension.rss_aux import RssSensor


class RssBasicAgent(BasicAgent):
	def __init__(self, vehicle, target_speed, rss_params):
		
		super(RssBasicAgent, self).__init__(vehicle, target_speed)

		self._rss_sensor = RssSensor(vehicle, rss_params)
		self._restrictor = carla.RssRestrictor()
		self._physics_control_static = vehicle.get_physics_control()


	def run_step(self, debug=False):
		# call basic agent control first
		control = super(RssBasicAgent, self).run_step()
		#-------------------------------------
		if self._restrictor:
			rss_restriction = self._rss_sensor.acceleration_restriction if self._rss_sensor and self._rss_sensor.response_valid else None
			if rss_restriction:
				#carla.VehiclePhysicsControl()
				vehicle_control_rss = self._restrictor.restrictVehicleControl(control, 
																			  rss_restriction, 
																			  self._rss_sensor.ego_velocity, 
																			  self._physics_control_static)
				#if not (control == vehicle_control_rss):
				#	print('RSS restrictor is ON: brake=%.3f, steer=%.3f' % (vehicle_control_rss.brake, vehicle_control_rss.steer))
				control = vehicle_control_rss
		#-------------------------------------
		return control