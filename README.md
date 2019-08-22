# RGT (Robusntess-guided testing)
RSS, CARLA, parameter exploration and magic.

# Usage
## Installation
- Get [CARLA 0.9.6](https://github.com/carla-simulator/carla/). Be sure the branch you chose supports RSS.
- If you need to, build carla and PythonAPI with RSS flag 
```
make launch
make package
make libCarla.client.rss
make PythonAPI.rss
```
- If there is an error "not compliant with RSS version, please do PythonAPI.rss" do
```
make clean
make PythonAPI.rss
```

- Define your PYTHONPATH:
```
export PYTHONPATH=path/to/your/carla/dist/.egg and path/to/your/carla/PythonAPI/carla
```
- Install [Scenario Runner](https://github.com/carla-simulator/scenario_runner)  (compatible version  with CARLA 0.9.6)
- Define the scenario_runner path:

```
export ROOT_SCENARIO_RUNNER=path/to/your/scenario_runner
```
- Git clone this code

## Launch 
Depending on your CARLA branch version the launching steps are as follows

**Branch: feature/rss** | **Branch: feature/rss-ext**
--- | --- 
-Run carla server with all required flags for this setting from your carla_folder:<br/>`./CarlaUE4.sh /Game/Carla/Maps/Town01 -benchmark -fps=20`<br/><br/>- Run rss_runner.py from RGT_folder<br/>`python rss_runner.py`<br/><br/><br/><br/><br/><img width=1000/>|- Run carla server with a benchmark flag from your carla_folder:<br/>`./CarlaUE4.sh -benchmark`<br/><br/>- Change carla server with required flags for this setting from carla_folder/PythonAPI/util:<br/>`./config.py -m Town01 --fps 20`<br/><br/>- Run rss_runner.py from RGT_folder<br/>`python rss_runner.py`<br/><img width=200/>
