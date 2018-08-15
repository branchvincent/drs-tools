'''
Cooperative Multi-Robot Observation of Moving Targets
'''

import matplotlib.pyplot as plt
import numpy as np
import numpy.polynomial.polynomial as poly

from argparse import ArgumentParser
from agents import Robot, Target
from math import ceil
from random import seed

from sim import Simulation

def run_ratios(ratios, tracking=False, samples=5, fname=None):
    '''Runs and plots each target-robot ratio'''
    radii = np.linspace(100, 500, 9)
    radii_plt = np.linspace(100, 500, 401)
    # Run simulation for each ratio
    for i, ratio in enumerate(ratios):
        print 'Ratio {} of {}: {}'.format(i+1, len(ratios), ratio)
        # Run each ratio for the given number of samples
        r, o = np.zeros(len(radii)), np.zeros(len(radii))
        for j in range(samples):
            print '\tSample {} of {}'.format(j+1, samples)
            rn, on = run_ratio(ratio, radii, tracking)
            r, o = np.add(r, rn), np.add(o, on)
        # Fit polynomial to average
        r, o = r/float(samples), o/float(samples)
        coefs = poly.polyfit(r, o, 3)
        o_fit = poly.polyval(radii_plt, coefs)
        plt.plot(radii_plt, o_fit, label='{}'.format(ratio))
    # Create plot
    plt.title('Observations vs. Radius')
    plt.xlabel('Environment Radius (m)')
    plt.ylabel('Average Targets Observed (%)')
    plt.legend(title='Ratio')
    plt.grid(True)
    # Save, if requested
    if fname:
        plt.savefig(fname)
    plt.show()

def run_ratio(ratio, radii, tracking, coverage=0.1, T=120, dt=1, fname=None):
    '''Runs the specified target-robot ratio for the specified environment radii'''
    # Run simulation for each radius
    observations = []
    for i, R in enumerate(radii):
        m_max, n_max = 10, 20
        if ratio <= 1:
            m = m_max
            n = int(ratio * m_max)
        else:
            n = n_max
            m = int(n_max/ratio)
        print '\tSim {} of {}: n = {}, m = {}'.format(i+1, len(radii), n, m)
        sim = Simulation(m, n, T, dt, R, tracking=tracking)
        sim.run(vis='', fname=fname)
        observations.append(sim.average_observations(normalize=True))
    return radii, observations

def parser():
    '''Creates the argument parser'''
    parser = ArgumentParser(description='Potential field control for cooperative multi-robot observation of moving targets')
    parser.add_argument('-s', '--seed', default=4, type=int, help='seed for random generator')
    parser.add_argument('-o', '--output_file', default=None, help='file destination of output')
    parser.add_argument('-m', default=3, type=int, help='# robots')
    parser.add_argument('-n', default=6, type=int, help='# targets')
    parser.add_argument('-t', default=120, type=int, help='total time')
    parser.add_argument('-dt', default=1, type=float, help='time step')
    parser.add_argument('-r', default=100, type=int, help='environment radius')
    parser.add_argument('-k', '--tracking', action='store_true', help='enable predictive tracking')
    parser.add_argument('-v', '--visualization', choices=['animate', 'plot', 'both'], default='animate', help='visualize simulation')
    parser.add_argument('-a', '--run_all', action='store_true', help='run simulation for every ratio')
    return parser

def main():
    # Read arguments
    args = parser().parse_args()
    seed(args.seed)
    # Run once
    if not args.run_all:
        sim = Simulation(args.m, args.n, args.t, args.dt, args.r, tracking=args.tracking)
        sim.run(vis=args.visualization, fname=args.output_file)
        print 'observations = {}'.format(sim.average_observations(normalize=True))
    # Run ratios
    else:
        ratios = [1/5., 1/2., 1, 4, 10]
        run_ratios(ratios, tracking=args.tracking, fname=args.output_file)

if __name__ == '__main__':
    main()
