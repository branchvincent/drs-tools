# Collision Avoidance
There is a 4-way intersection with randomly spawning robots attempting to cross. The robots attempt to travel straight across the intersection as quickly as possible without leaving the road and without colliding with others.

## Method
The collision avoidance method used is described [here](https://doi.org/10.1007/978-3-642-19457-3_1) and implemented in the third-party library [RVO2](https://github.com/sybrenstuvel/Python-RVO2).

## Setup
Python 2.7.13 was chosen as the programming language for this assignment. In addition to Python, the `RVO2` library as well as the `numpy` and `matplotlib` packages are required. The library can be cloned from [here](https://github.com/sybrenstuvel/Python-RVO2) and installed following the repository's `README`. The packages can be installed via the following command:
```python
pip install -r requirements.txt
```

## Simulation
The simulation can be run via `python simulate.py`, which will run the default scenario. For more options, you can pass the flag `-h`.
