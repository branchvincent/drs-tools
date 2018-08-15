from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

import numpy as np
import matplotlib.pyplot as plt

from agents import Robot, Target

class Simulation(object):
    '''
    Simulation of robots tracking targets

    Inputs:
        - robots: robots to cooperatively track targets
        - targets: targets to be tracked
        - env_radius: environment radius (m)
    '''
    def __init__(self, m, n, T, dt, env_radius, tracking=False):
        # Create agents
        self.robots = [Robot(env_radius, tracking=tracking) for _ in range(m)]
        self.targets = [Target(env_radius) for _ in range(n)]
        for agent in self.robots + self.targets:
            agent.set_robots(self.robots)
            agent.set_targets(self.targets)
        # Create environment
        self.T, self.dt = float(T), float(dt)
        self.ts = np.arange(0, self.T, self.dt)
        self.env = Circle((0, 0), env_radius, color='black', linewidth=3, alpha=0.25)
        self.observed_targets = 0

    def _init_ani_fig(self):
        '''Initialize the animation's figure'''
        # Create plot
        self.fig, self.ax = plt.subplots()
        # Set axes
        r = self.env.radius
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        self.ax.set_aspect(1)
        self.ax.add_patch(self.env)
        # Set time label
        self.time_label = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        plt.title('Potential Field Control')

    def _control_loop(self):
        '''Control loop for the agents'''
        # Update control
        agents = self.robots + self.targets
        for agent in agents:
            agent.update_control(self.dt)
        # Calculate A
        for target in self.targets:
            if target.sensed():
                self.observed_targets +=1
        # Update state
        for agent in agents:
            agent.update_state(self.dt)

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
        for agent in self.robots + self.targets:
            drawables.extend(agent.draw(self.ax))
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
        plt.show()

    def _run_sim(self):
        '''Run the simulation only'''
        for _ in self.ts:
            self._control_loop()

    def run(self, vis='animate', fname=None):
        '''Run the simulation with the requested visualization'''
        if vis == 'both':
            self._run_ani(fname=fname)
            self.plot(fname=fname)
        elif vis == 'animate':
            self._run_ani(fname=fname)
        elif vis == 'plot':
            self._run_sim()
            self.plot(fname=fname)
        else:
            self._run_sim()
        return self.average_observations()

    def average_observations(self, normalize=True):
        '''The average number of observed targets'''
        avg = self.observed_targets/self.T
        if normalize and len(self.targets) != 0:
            return avg/len(self.targets)
        return avg

    def plot(self, fname=None, ext='png'):
        '''Plot the agents' simulated paths'''
        # Initialize plot
        fig, ax = plt.subplots()
        r = self.env.radius
        ax.set_xlim(-r, r)
        ax.set_ylim(-r, r)
        ax.set_aspect(1)
        ax.add_patch(self.env)
        # Plot robot data
        robot_xdata = [x[0] for robot in self.robots for x in robot.state_history]
        robot_ydata = [x[1] for robot in self.robots for x in robot.state_history]
        plt.plot(robot_xdata, robot_ydata, 'bo', alpha=0.4, markersize=3, label='robot')
        for robot in self.robots:
            c, r = robot.state_history[0][:2], robot.sensing_range()
            circle = Circle(c, r, ec='black', fc='none', linewidth=3, alpha=0.25)
            ax.add_patch(circle)
        # Plot target data
        target_xdata = [x[0] for target in self.targets for x in target.state_history]
        target_ydata = [x[1] for target in self.targets for x in target.state_history]
        plt.plot(target_xdata, target_ydata, 'cs', alpha=0.4, markersize=3, label='target')
        # Add legend
        ax.legend(numpoints=1, loc='upper center', bbox_to_anchor=(0.5, -0.03), ncol=2)
        time_txt = 'T = {:.3g} s\ndt = {:.3g}'.format(self.T, self.dt)
        ax.text(0.02, 0.95, time_txt, transform=ax.transAxes, va='top')
        ax.text(0.87, 0.9, 'm = {:.3g}\nn = {:.3g}'.format(len(self.robots), len(self.targets), ha='left', va='right'), transform=ax.transAxes)
        # Save, if requested
        if fname:
            plt.savefig('{}.{}'.format(fname, ext))
        plt.show()
