# Cooperative Multi-Robot Observation of Moving Targets

There are m point robots whose goal is to track n targets. More specifically, the holonomic point robots have 360-degree field-of-view sensors of limited range and the holonomic targets move randomly within a circular environment. The speed of each target is fixed and randomly chosen. The robots try to maximize the average number of targets that are being observed by at least one robot at each time step.

## Method
The tracking method used is a potential field control, where the robots are repulsed by each other and attracted to targets. 

## General Setup
Python 2.7.13 was chosen as the programming language for this assignment. In addition to Python, the `numpy` and `matplotlib` packages are also required. These can be installed, if not already, via the following command:
```python
pip install -r requirements.txt
```

## Simulation
The simulation can be run via `python simulate.py`, which will run the default scenario. For more options, you can pass the flag `-h`.
