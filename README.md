# RGT (Robusntess-guided testing)
RSS, CARLA, parameter exploration and magic.

# Usage
## Installation
- get CARLA 0.9.6
- If you need to, build carla and PythonAPI with RSS flag 
```
make launch
make libCarla.client.rss
make PythonAPI.rss
make package

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
- Install Scenario Runner (compatible version  with CARLA 0.9.6)
- Define the scenario_runner path:

```
export ROOT_SCENARIO_RUNNER=path/to/your/scenario_runner
```
- Git clone this code

## Launch 
- Run carla server with required flags for this setting from your carla_folder 
```
./CarlaUE4.sh -benchmark
```

- Change carla server with required flags for this setting from carla_folder/PythonAPI/util: 
```
./config.py -m Town01 --fps 20

```
- Run rss_runner.py from RGT_folder:
```
python rss_runner.py
```
