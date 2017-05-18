import numpy as np


class PV:
    """
    PV (Photovoltaic) system class

    Configuration:
        Divides the

    Attributes:
        A (int): Solar panel A in m^2
        hour (int): Hour from 0 to 23
        H (float): Solar radiance per m^2
        R (float): Solar panel yield
        PR (float): performance ratio (default 0.75)
    """

    def __init__(self, area):
        # TODO: Find a fitting area depending on the energy requirement
        # of the system

        # pprint('Initializing Photovoltaic source')
        self.A = area

        self.hour = 0
        self.E = 0
        self.H = 0

        # for Davao City, panel facing East at 83 deg angle
        self.H_max = 501.9
        self.R = 0.15
        self.PR = 0.75

    def get_hour_H(self):
        return self.H_max * abs(np.sin(self.hour * np.pi / 24))

    def get_E(self):
        self.hour += 1
        self.hour = self.hour % 24
        self.H = self.get_hour_H()

        self.E = self.A * self.R * self.PR * self.H * 3600  # Wh to Joules

        return self.E
