from math import pi, sin, cos
import numpy as np
from numpy.linalg import norm
from matplotlib.patches import Circle, Arrow
from util import pol2cart, cart2pol, sign, unit_vec, angle, reflect, tangent_vec
from random import random, uniform

from abc import ABCMeta, abstractmethod

def close(x, y, eps=0.001):
    return norm(np.subtract(x, y)) <= eps

class Agent(object):
    '''
    Base clase for an agent (2D point) in the simulation.

    Inputs:
        - x: state, consisting of position (m) and orientation (radians)
        - u: control, consisting of speed (m/s) and relative steering angle (radians)
        - env_radius: radius of environment radius (m)
        - radius: radius of agent
        - color: color of agent
    '''
    __metaclass__ = ABCMeta

    def __init__(self, x, u, env_radius=100, radius=2, color='black'):
        self._x, self._u = list(x), list(u)
        self.body = Circle(self.position(), radius, color=color)
        self.env_radius = env_radius
        self.drawables = [self.body]
        self.state_history = [self._x[:]]
        self.robots, self.targets = [], []

    def position(self):
        '''Position of agent'''
        return self._x[0:2]

    def orientation(self):
        '''Orientation of agent'''
        return self._x[2]

    def speed(self):
        '''Speed of agent'''
        return self._u[0]

    def steering(self):
        '''Relative steering angle of agent'''
        return self._u[1]

    def set_position(self, pos):
        '''Sets agent's position'''
        self._x[0:2] = pos
        # assert(self.position() == self._x[0:2])

    def set_orientation(self, theta):
        '''Sets agent's orientation'''
        # assert(not hasattr(theta, '__iter__'))
        self._x[2] = theta
        # assert(close(self.orientation(), theta))

    def set_speed(self, speed):
        '''Sets agent's speed'''
        self._u[0] = speed

    def set_steering(self, alpha):
        '''Sets agent's relative steering angle'''
        self._u[1] = alpha

    def set_robots(self, robots):
        '''Sets the robots in the environment'''
        self.robots = [robot for robot in robots if robot != self]

    def set_targets(self, targets):
        '''Sets the targets in the environment'''
        self.targets = [target for target in targets if target != self]

    def orientation_vec(self, mag=1):
        '''Orientation vector of the agent'''
        t = pol2cart((mag, self.orientation()))
        return pol2cart((mag, self.orientation()))

    def distance(self, agent):
        '''Euclidean distance between two agents'''
        diff = np.subtract(agent.position(), self.position())
        return norm(diff)

    def out_of_bounds(self):
        '''Returns if the agent is out of bounds'''
        return cart2pol(self.position())[0] >= self.env_radius

    def heading_in_bounds(self):
        '''Returns if the agent is heading back in bounds'''
        return abs(angle(self.position(), self.orientation_vec())) > pi/2

    def update_control(self, dt):
        '''Updates the control'''
        # Update heading, if out of bounds
        if self.out_of_bounds() and not self.heading_in_bounds():
            neg_pos = [-p for p in self.position()]
            self.set_steering(reflect(self.orientation_vec(), neg_pos))
            # self.set_steering(-sign(self.orientation()) * pi)
            # print 'WARNING: steering to {}'.format(self._u[1])

    def update_state(self, dt):
        '''Updates the state using the system's dynamics'''
        # Update state
        self._x[0] += self.speed() * cos(self.orientation()) * dt
        self._x[1] += self.speed() * sin(self.orientation()) * dt
        self._x[2] += self.steering()
        # Reset control and update state history
        self.set_steering(0)
        self.state_history.append(self._x[:])

    def draw(self, ax):
        '''Draws the agent at each time step'''
        # Draw everything
        drawn_items = []
        for drawable in self.drawables:
            if isinstance(drawable, Circle):
                drawable.center = self.position()
            else:
                print 'WARNING: unrecognized drawable'
            drawn_items.append(ax.add_patch(drawable))
        # Add orientation arrow
        # orient_arrow = Arrow(*(self.position() + list(self.orientation_vec(5))), width=1.0, color='red', alpha=0.25)
        # drawn_items.append(ax.add_patch(orient_arrow))
        return drawn_items


class Robot(Agent):
    '''
    Robot capable of tracking targets.

    Inputs: (see Agent)
        - max_speed: maximum speed (m/s)
    '''
    def __init__(self, env_radius=100, x=None, u=None, max_speed=2, tracking=False, color='blue'):
        self.max_speed = max_speed
        # Set x, u if needed
        x = x or pol2cart((uniform(0, env_radius), uniform(-pi, pi))) + tuple([uniform(-pi, pi)])
        u = u or [self.max_speed, 0]
        super(Robot, self).__init__(x, u, env_radius=env_radius, color=color)
        # Initialize potential field
        self.predictive_tracking = tracking
        if self.predictive_tracking:
            self.Frt_p = ((0, -1), (4, 0), (8, 1), (25, 1), (30, 0))
        else:
            self.Frt_p = ((0, -1), (4, 0), (8, 1), (30, 1), (30, 0))
        self.Frr_p = ((0, -1), (12.5, -1), (20, 0))
        do, _ = zip(*self.Frt_p)
        dr, _ = zip(*self.Frr_p)
        # Add drawables
        radii = do[1:] if self.predictive_tracking else do[1:-1]
        colors = ['red', 'green', 'green', 'blue']
        for c,r in zip(colors, radii):
            self.drawables.append(Circle(self.position(), r, ec=c, fc='none', linewidth=1, alpha=0.25))
        # Set targets
        self.sensed_targets, self.tracked_targets = [], []

    def sensing_range(self):
        '''Maximum sensing range of the robot'''
        return self.Frt_p[3][0]

    def tracking_range(self):
        '''Maximum predictive tracking range of the robot'''
        return self.Frt_p[4][0]

    def sensing(self, target):
        '''Returns if the target is being sensed'''
        return self.distance(target) <= self.sensing_range()

    def in_tracking_range(self, target):
        '''Returns if the target is being tracked'''
        if not self.predictive_tracking:
            return False
        return self.sensing_range() <= self.distance(target) <= self.tracking_range()

    def idle(self):
        '''Returns if the robot is not tracking any target'''
        for target in self.targets:
            if self.sensing(target) or (target in self.tracked_targets):
                return False
        return True

    def weight(self, target):
        '''Weight for the potential field'''
        return 1 if self.sensing(target) and target.num_sensing() <= 1 else 0.25

    # def potential_field(self):
    #     '''Calculates the potential field for the robot'''
    #     # Calculate force component for each target and robot
    #     force = np.zeros(len(self.position()))
    #     for target in self.targets:
    #         force = np.add(force, self.weight(target) * self.target_force(target))
    #     for robot in self.robots:
    #         force = np.add(force, self.robot_force(robot))
    #     # Normalize to have magnitude of max_speed
    #     return (self.max_speed * unit_vec(force)).tolist()

    def potential_field(self, dt):
        '''Calculates the potential field for the robot'''
        # Calculate force component for each target and robot
        force = np.zeros(len(self.position()))
        if self.predictive_tracking:
            # Update sensed and tracked targets
            for target in self.sensed_targets:
                if not self.sensing(target):
                    self.sensed_targets.remove(target)
                if self.in_tracking_range(target):
                    self.tracked_targets.append(Target(env_radius=self.env_radius, x=target._x[:], u=target._u[:]))
            for target in self.tracked_targets:
                target.update_state(dt)
                if not self.in_tracking_range(target):
                    self.tracked_targets.remove(target)
            # Add new tracked
            for target in self.targets:
                if self.sensing(target) and not (target in self.sensed_targets):
                    self.sensed_targets.append(target)
            for target in self.sensed_targets + self.tracked_targets:
                force = np.add(force, self.weight(target) * self.target_force(target))
            for robot in self.robots:
                force = np.add(force, self.robot_force(robot))
        else:
            for target in self.targets:
                force = np.add(force, self.weight(target) * self.target_force(target))
            for robot in self.robots:
                force = np.add(force, self.robot_force(robot))
        # Normalize to have magnitude of max_speed
        return (self.max_speed * unit_vec(force)).tolist()

    def robot_force(self, robot):
        '''Force vector for robot'''
        difference = np.subtract(robot.position(), self.position())
        f_mag = np.interp(norm(difference), *zip(*self.Frr_p))
        return f_mag * difference

    def target_force(self, target):
        '''Force vector for target'''
        difference = np.subtract(target.position(), self.position())
        f_mag = np.interp(norm(difference), *zip(*self.Frt_p))
        return f_mag * difference

    def update_control(self, dt):
        '''Updates the control'''
        self.force = self.potential_field(dt)
        # Update speed, depending on status
        if self.idle():
            self.set_speed(self.max_speed)
        else:
            self.set_speed(norm(self.force))
        # Update steering angle
        if norm(self.force) != 0:
            self.set_steering(angle(self.orientation_vec(), self.force))
        # Update heading, if out of bounds
        super(Robot, self).update_control(dt)

    def draw(self, ax):
        '''Draws the robot, called every time step'''
        drawables = super(Robot, self).draw(ax)
        force = np.multiply(10, self.force).tolist()
        # force_arrow = Arrow(*(self.position() + force), width=1.0, color='blue', alpha=0.25)
        # drawables.append(ax.add_patch(force_arrow))
        return drawables

class Target(Agent):
    '''
    Target to be tracked.

    Inputs: (see Agent)
    '''
    def __init__(self, env_radius=100, x=None, u=None, color='cyan'):
        # Set x, u if needed
        x = x or pol2cart((uniform(0, env_radius), uniform(-pi, pi))) + tuple([uniform(-pi, pi)])
        u = u or [uniform(0, 1.5), 0]
        super(Target, self).__init__(x, u, env_radius, color=color)

    def sensed(self):
        '''Returns if the target is being tracked'''
        for robot in self.robots:
            if robot.sensing(self):
                return True
        return False

    def num_sensing(self):
        '''The number of robots sensing the target'''
        n = 0
        for robot in self.robots:
            if robot.sensing(self):
                n += 1
        return n

    def num_tracking(self):
        '''The number of robots tracking the target'''
        n = 0
        for robot in self.robots:
            if robot.tracking(self):
                n += 1
        return n

    def update_control(self, dt, rand=True, chance=0.05):
        '''Updates the control'''
        # Maintain speed and randomly steer
        if rand:
            alpha = uniform(-pi/2, pi/2) if random() < chance else 0
            self.set_steering(alpha)
        # Update heading, if out of bounds
        super(Target, self).update_control(dt)
