# RSS Extended
A set of CARLA scenarios demonstrating expanded scope for Responsibility-Sensitive Safety (RSS).  

## Background

This repository is a fork of robustness-guided testing, [https://github.com/nellro/rgt](https://github.com/nellro/rgt)  .

It uses CARLA [Scenario Runner](https://github.com/carla-simulator/scenario_runner) . 

## Installation


1. Clone the latest [CARLA feature/rss-0.96](https://github.com/carla-simulator/carla/tree/feature/rss-0.9.6) branch. 

2. Install [CARLA build tools, dependencies and Unreal Engine](https://carla.readthedocs.io/en/latest/how_to_build_on_linux/)

3. Build CARLA and PythonAPI with RSS flag 

        make launch
        make package 
        make libCarla.client.rss
        make PythonAPI.rss

    * If there is an error "not compliant with RSS version, please do PythonAPI.rss" do

            make clean
            make PythonAPI.rss

4. Define your PYTHONPATH:

        export PYTHONPATH=$PYTHONPATH:/path/to/your/carla/dist/<name of dist>.egg
        export PYTHONPATH=$PYTHONPATH:/path/to/your/carla/PythonAPI/carla
        
5. Install [Scenario Runner](https://github.com/carla-simulator/scenario_runner)  (compatible version  with CARLA 0.9.6)

6. Define the scenario_runner path:

        export ROOT_SCENARIO_RUNNER=path/to/your/scenario_runner

7. Git clone this code

8.  Add the `code` directory of this repo to your python path:

        export PYTHONPATH=$PYTHONPATH:/path/to/rss-extended/code

## Launch 

1. Run 

          make launch

     to start the CARLA UE4 editor.  

2. In the editor window, press the Play button (large triangle).  Unreal Engine will now wait for incoming connections from the Python API.

3. In this repository, using Python3, 
 
        cd code
        python rss_runner.py --scenario ScenarioName
        
    using a scenario name from the section below.  If `--scenario ScenarioName` is omitted, the first scenario will be run.  For example, 
    
        cd code
        python rss_runner.py --scenario Rss_Ext_FI_a
        
will run the forward incursion scenario, variant a.  
        
If you see an error:

        AttributeError: 'ScenarioRunner' object has no attribute '_start_time'

the scenario may have timed out waiting for the Unreal Engine.  Please check that "Play" has been pressed in the Unreal Editor and try again until success.

## Scenarios

This list is also included in `rss_runner.py`.

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



