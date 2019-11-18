import carla
from agents.navigation.basic_agent import BasicAgent
from scenario_runner_extension.rss_aux import RssSensor


class RssExtAgent(BasicAgent):
	def __init__(self, vehicle, target_speed, rss_params, variant):

		super(RssExtAgent, self).__init__(vehicle, target_speed)

		# For self-certifying behavior, assume a max_accel of 0.1,
		# better max braking of 0.8, and better response time of 0.1

		self._variant = variant	# Scenario variant
		if (self._variant == 'Rss_Ext_SFD_b'):
			g = 9.8
			rss_params['alpha_lon_accel_max'] = 0.05*g
			rss_params['alpha_lon_brake_min'] = 0.7*g
			rss_params['response_time'] = 0.05


		if (self._variant == 'Rss_Ext_PLAT_b'):
			g = 9.8
			rss_params['alpha_lon_accel_max'] = 0.05*g
			rss_params['alpha_lon_brake_min'] = 0.7*g
			rss_params['response_time'] = 0.05

		self._rss_sensor = RssSensor(vehicle, rss_params)
		self._restrictor = carla.RssRestrictor()
		self._physics_control_static = vehicle.get_physics_control()


	def run_step(self, debug=False):
		# call basic agent control first
		control = super(RssExtAgent, self).run_step()

		#-------------------------------------
		if self._restrictor:
			rss_restriction = self._rss_sensor.acceleration_restriction if self._rss_sensor and self._rss_sensor.response_valid else None
			if rss_restriction:

				# This returns a carla.VehicleControl object, with fields:
				# VehicleControl(throttle=1.000000, steer=-1.000000, brake=0.000000, hand_brake=False, reverse=False, manual_gear_shift=False, gear=0)
				vehicle_control_rss = self._restrictor.restrictVehicleControl(control,
																			  rss_restriction,
																			  self._rss_sensor.ego_velocity,
																			  self._physics_control_static)

			    # Variant RSS_Ext_FI_a - Cut-in vehicle at constant velocity - Ego vehicle applies gentle braking profile of half of RSS Classic braking
				if (self._variant == 'Rss_Ext_FI_a'):
					print ('Applying RSS Classic braking profile, scenario RSS_Ext_FI_a')

				# Variant RSS_Ext_FI_b - Cut-in vehicle braking - Ego vehicle applies RSS Classic braking profile
				elif (self._variant == 'Rss_Ext_FI_b'):
					print('Applying RSS Extended gentle braking profile, scenario RSS_Ext_FI_b')
					vehicle_control_rss.brake = vehicle_control_rss.brake / 2.0

				# Variant RSS_Ext_SI_a - Side incursion - Ego vehicle brakes due to obstruction
				elif (self._variant == 'Rss_Ext_SI_a'):
					print('Applying RSS Classic braking profile which will stop ego vehicle, scenario RSS_Ext_SI_a')

				elif (self._variant == 'Rss_Ext_SI_b'):
					print('Applying RSS Extended in-lane evasive manuever, scenario RSS_Ext_SI_b')
					if not (control == vehicle_control_rss):
						vehicle_control_rss.steer = -0.2

				elif (self._variant == 'Rss_Ext_SI_c'):
					print('Applying RSS Extended lane change evasive maneuver, scenario RSS_Ext_SI_c')
					if not (control == vehicle_control_rss):
						vehicle_control_rss.steer = -0.4

				elif (self._variant == 'Rss_Ext_SFD_a'):
					print('Applying RSS Classic Following Distance, scenario RSS_Ext_SFD_a')

				elif (self._variant == 'Rss_Ext_SFD_b'):
					print('Applying RSS Extended Shorter Following Distance, scenario RSS_Ext_SFD_b')

				elif (self._variant == 'Rss_Ext_MERGE_a'):
					print('Applying RSS Classic lateral restriction, scenario RSS_Ext_MERGE_a')

				elif (self._variant == 'Rss_Ext_MERGE_b'):
					print('Applying RSS Classic - Extra lateral restriction, scenario RSS_Ext_MERGE_b')
					if (vehicle_control_rss.steer < 0):
						vehicle_control_rss.steer = -0.6
					else:
						vehicle_control_rss.steer = 0.04

				elif (self._variant == 'Rss_Ext_MERGE_c'):
					print('Applying RSS Extended merge, scenario RSS_Ext_MERGE_c')
					vehicle_control_rss = control

				# Next - update restrictor based on lead vehicle speed, etc.
				if not (control == vehicle_control_rss):
					print('RSS restrictor is ON: brake=%.3f, steer=%.3f' % (vehicle_control_rss.brake, vehicle_control_rss.steer))
				control = vehicle_control_rss

		#-------------------------------------
		return control
