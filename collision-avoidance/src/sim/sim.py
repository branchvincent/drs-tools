from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from random import random, choice, uniform
from itertools import combinations
from numpy.linalg import norm
from util import *

import rvo2
import numpy as np
import matplotlib.pyplot as plt

class Simulation(object):
    '''
    Simulation of robots avoiding collisions

    Inputs:
        - prob: robots to cooperatively track targets
        - vmax: targets to be tracked
        - r: robot radius (m)
        - R: environment radius (m)
        - T: end time (s)
        - dt: time step (s)
        - turning: enable robots to turn at the intersection
    '''
    def __init__(self, prob, vmax=20, w=7, r=2, R=200, T=60, dt=0.2, turning=False):
        self.prob = float(prob)
        self.vmax = float(vmax)
        self.w = float(w)
        self.T, self.dt = float(T), float(dt)
        self.ts = np.arange(0, self.T, self.dt)
        self.r, self.R = r, R
        self.turning = turning
        self.collisions = 0
        self.not_acted = set()
        self.actions = []
        self.bodies = []
        self._init_rvo()

    def _init_rvo(self, neighborDist=1.5, maxNeighbors=5, timeHorizon=1.5, timeHorizonObst=2):
        self.sim = rvo2.PyRVOSimulator(self.dt, neighborDist, maxNeighbors,
                                       timeHorizon, timeHorizonObst, self.r, self.vmax)

    def _init_ani_fig(self):
        '''Initialize the animation's figure'''
        # Create plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-self.R, self.R)
        self.ax.set_ylim(-self.R, self.R)
        self.ax.set_aspect(1)
        # Draw environment
        self.ax.add_patch(Circle((0, 0), self.R, color='black', linewidth=3, alpha=0.25))
        # Draw roads and lanes
        xy, wh = (-self.R, -self.w/2), (2*self.R, self.w)
        col, lw, a = 'white', 0, 1
        self.ax.add_patch(Rectangle(xy, *wh, color=col, linewidth=lw, alpha=a))
        self.ax.add_patch(Rectangle(list(reversed(xy)), *list(reversed(wh)), color=col, linewidth=lw, alpha=a))
        xx, yy = (-self.R, self.R), (0, 0)
        col, lw = 'blue', 0.5
        self.ax.plot(xx, yy, '--', color=col, linewidth=lw)
        self.ax.plot(yy, xx, '--', color=col, linewidth=lw)
        # Set time label
        self.time_label = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        plt.title('Collision Avoidance')

    def _control_loop(self):
        '''Control loop for the agents'''
        # Update agents
        self.sim.doStep()
        # Add new agent, if applicable
        if random() <= self.prob:
            # Determine entrance
            entrances = [(self.R, 0), (0, self.R), (-self.R, 0), (0, -self.R)]
            entrance = choice(entrances)
            # Determine position and velocity
            theta0 = np.arctan2(*reversed(entrance))
            theta = np.arctan(self.w/(2*self.R))
            pos = pol2cart((self.R, theta0 + theta/2))
            vel_raw = [-i for i in entrance]
            vel = tuple(scale(vel_raw, self.vmax))
            # Determine turning action
            actions = ['straight', 'left', 'right']
            probabilities = [0.5, 0.25, 0.25]
            action, = np.random.choice(actions, 1, p=probabilities)
            self.actions.append(action)
            # Set attributes
            agent = self.sim.addAgent(pos)
            self.sim.setAgentPrefVelocity(agent, vel)
            self.not_acted.add(agent)
            # Create body for sim
            colors = ['r', 'g', 'b', 'm', 'k']
            color = colors[agent%len(colors)]
            self.bodies.append(Circle(pos, self.r, color=color))
        # Check for collisions
        N = self.sim.getNumAgents()
        for a1, a2 in combinations(range(N), 2):
            p1 = self.sim.getAgentPosition(a1)
            p2 = self.sim.getAgentPosition(a2)
            diff = norm(np.subtract(p1, p2))
            if diff <= 2*self.r:
                self.collisions += 1
                # print 'COLLSION'
        # Check for turning
        if self.turning:
            directions = {'straight': 0, 'left': np.pi/2, 'right': -np.pi/2}
            acted_agents = []
            for agent in self.not_acted:
                # Get agent attributes
                pos = self.sim.getAgentPosition(agent)
                vel0 = self.sim.getAgentPrefVelocity(agent)
                action = self.actions[agent]
                direction = directions[action]
                # Determine when to steer robot
                if action == 'straight':
                    ready = True
                elif action == 'left':
                    crossed_int = abs(angle(pos, vel0)) <= np.pi/2
                    time_to_cross = norm(pos) >= norm([self.w/4, self.w/4])
                    ready = crossed_int and time_to_cross
                elif action == 'right':
                    time_to_cross = norm(pos) <= norm([self.w/4, self.w/4])
                    ready = time_to_cross
                # Steer robot, if applicable
                if ready:
                    vel = tuple(rotate(vel0, direction))
                    self.sim.setAgentPrefVelocity(agent, vel)
                    acted_agents.append(agent)
            # Removed acted agents
            self.not_acted.difference_update(acted_agents)

    def _init_ani(self):
        '''Initialize the animation'''
        # Initialize drawables and time label
        drawables = []
        self.time_label.set_text('')
        return (self.time_label, )  + tuple(drawables)

    def _update_ani(self, t, dt):
        '''Update the animation'''
        # Update agents
        self._control_loop()
        # Update plot
        drawables = []
        for agent, body in enumerate(self.bodies):
            pos = self.sim.getAgentPosition(agent)
            body.center = pos
            drawables.append(self.ax.add_patch(body))
        # Update time label
        self.time_label.set_text('t = {:.3g} s'.format(t))
        return (self.time_label, ) + tuple(drawables)

    def _run_ani(self, fname=None, ext='mp4'):
        '''Run the animation'''
        self._init_ani_fig()
        ani = FuncAnimation(self.fig, self._update_ani, fargs=[self.dt], frames=self.ts, init_func=self._init_ani, blit=True, repeat=False, interval=200)
        # Save video, if requested
        if fname:
            ani.save('{}.{}'.format(fname, ext), writer='ffmpeg', fps=30)
        else:
            plt.show()

    def _run_sim(self):
        '''Run the simulation only'''
        for _ in self.ts:
            self._control_loop()

    def run(self, animate=True, fname=None):
        '''Run the simulation with the requested animation'''
        if animate:
            self._run_ani(fname=fname)
        else:
            self._run_sim()
        return self.average_collisions()

    def average_collisions(self):
        '''The average number of observed targets'''
        N = self.sim.getNumAgents()
        return self.collisions/float(N) if N != 0 else 0
