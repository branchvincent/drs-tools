# Robocup Team
This is a high-level design for a robot soccer team that could compete in the [RoboCup 3D Simulation League](http://www.robocup.org/leagues/25). This project focuses on formations, roles, and behaviors rather than low-level mechanics. 

## Method
Our method consists of 3 components: goalie, defense, and offense. See the [docs](https://github.com/branchvincent/drs-tools/tree/master/robocup-team/doc) for each component's specific strategy.

## Setup
Our code was adapted from UT Austin Villa's released C++ [codebase](https://github.com/LARG/utaustinvilla3d). This codebase can be installed (along with its dependencies) by following the repository's `README`. Then, our `naobehavior.h` and `strategy.cc` files need to replace the existing files found at `utaustinvilla3d/behaviors/`.  

## Simulation
The simulation can be run as detailed in the repository's `README`, without modification, via `./start.sh`.
