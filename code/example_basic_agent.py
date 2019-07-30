import random
import carla
import math
from agents.navigation.basic_agent import BasicAgent


def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def game_loop():
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(4.0)
    world = client.get_world()
    controller = carla.VehicleControl()

    blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
    start_pose = carla.Transform(carla.Location(107, 133, 0.7))      
    player = world.spawn_actor(blueprint, start_pose)

    agent = BasicAgent(player)
    agent.set_destination((326, 133, 0.7))

    # locate the view
    spectator = world.get_spectator()
    spectator.set_transform(get_transform(player.get_location(), 180))
    while True:
        try:
            world.wait_for_tick(10.0)
            world.tick()
            control = agent.run_step()
            player.apply_control(control)
        except Exception as exception:
            print("Exception")

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================
def main():
    try:
        game_loop()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
