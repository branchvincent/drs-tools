'''
Highway Collision Avoidance
'''

import matplotlib.pyplot as plt
import numpy as np

from argparse import ArgumentParser
from random import seed

from sim import Simulation

def run_all(probabilities, samples=25, turning=False, fname=None):
    average_collisions = []
    # Run simulation for each ratio
    for i, probability in enumerate(probabilities):
        print 'Probability {} of {}: {}'.format(i+1, len(probabilities), probability)
        # Run each probability for the given number of samples
        collisions = run(probability, samples=samples, turning=turning)
        average_collisions.append(collisions)
    # Plot
    plt.plot(average_collisions, probabilities)
    plt.title('Probability vs. Collisions')
    plt.xlabel('Average Collisions (per robot)')
    plt.ylabel('Probability of Robot Entering (%)')
    plt.grid(True)
    # Save, if requested
    if fname:
        plt.savefig(fname)
    plt.show()

def run(probability, samples=1, turning=False, fname=None):
    # Run simulation for each radius
    average_collisions = []
    for i, sample in enumerate(range(samples)):
        print '\tSim {} of {}'.format(i+1, samples)
        sim = Simulation(probability, turning=turning)
        sim.run(animate=False, fname=fname)
        average_collisions.append(sim.average_collisions())
    return float(sum(average_collisions)) / len(average_collisions)

def parser():
    '''Creates the argument parser'''
    parser = ArgumentParser(description='Highway Collision Avoidance')
    parser.add_argument('-s', '--seed', default=0, type=int, help='seed for random generator')
    parser.add_argument('-o', '--output_file', default=None, help='file destination of output')
    parser.add_argument('-p', '--probability', default=0.04, type=int, help='probability of robots entering')
    parser.add_argument('-u', '--turning', action='store_true', help='enable turning at intersection')
    parser.add_argument('-i', '--animate', action='store_true', help='animate simulation')
    parser.add_argument('-a', '--run_all', action='store_true', help='run simulation for every probability')
    return parser

def main():
    # Read arguments
    args = parser().parse_args()
    seed(args.seed)
    # Run once
    if not args.run_all:
        sim = Simulation(args.probability, turning=args.turning)
        sim.run(animate=args.animate, fname=args.output_file)
        print 'collisions = {}'.format(sim.average_collisions())
    # Run all
    else:
        probabilities = np.linspace(0.04, 0.2, 17)
        run_all(probabilities, turning=args.turning, fname=args.output_file)

if __name__ == '__main__':
    main()
