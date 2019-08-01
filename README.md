# RGT (Robusntess-guided testing)
RSS, CARLA, parameter exploration and magic.

# Usage
## Installation
- Install CARLA 0.9.5
- Install Scenario Runner (version compatible with CARLA 0.9.5)
- Git clone this code
- Define the scenario_runner path:
```
export ROOT_SCENARIO_RUNNER=path/to/your/scenario_runner
``` 


## Launch 
- Run carla server with required flags for this setting: 
```
./CarlaUE4.sh /Game/Carla/Maps/Town01 -benchmark -fps=20
```
- Run rss_runner.py:
```
python rss_runner.py
```
