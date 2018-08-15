import numpy as np
from numpy.linalg import norm

def sign(x):
    '''Returns sign of a number'''
    return -1 if x <= 0 else 1

def cart2pol(pos):
    '''Converts cartesian to polar coordinates'''
    x, y = pos
    theta = np.arctan2(y, x)
    rho = np.hypot(x, y)
    return rho, theta

def pol2cart(pos):
    '''Converts polar to cartesian coordinates'''
    rho, theta = pos
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    return x, y

def unit_vec(v):
    '''Returns normalized, unit vector'''
    if norm(v) == 0:
        return v
    else:
        return v/norm(v)

def angle(v1, v2):
    '''Angle between two vectors'''
    a = np.arctan2(*reversed(v2)) - np.arctan2(*reversed(v1))
    if not (-np.pi <= a <= np.pi):
        a -= sign(a) * 2 * np.pi
    return a

def tangent_vec(v):
    '''Tangent vector'''
    if norm(v) == 0:
        return [0, 1]
    return -v[1], v[0]

def reflect(v1, v2):
    '''Reflect v1 against the normal v2'''
    v2 = unit_vec(v2)
    ref = (v1 - 2 * v2 * np.dot(v1, v2)).tolist()
    return angle(v1, ref)
