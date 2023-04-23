# A code stimulate authors
from math import cos, sin

class Robot:
    
    def __init__(self, _position):
        self.position = _position
        self.theta = 0
    
    def motion_propagation(self, odometry_star_input, dt):
        [v_star, omega] = odometry_star_input

        self.theta = self.theta + omega * dt

        # true pose update
        self.position[0] += cos(self.theta) * v_star * dt
        self.position[1] += sin(self.theta) * v_star * dt