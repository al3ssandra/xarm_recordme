import numpy as np
from abc import ABC, abstractmethod


class WheeledMobileRobotInterface(ABC):
    @abstractmethod
    def __init__(self):
        pass


class WheelInterface(ABC):
    @abstractmethod
    def __init__(self):
        pass


class MobileManipulator(WheeledMobileRobotInterface):
    def __init__(self, joints, wheels):
        self._joints = joints
        self._wheels = wheels


class WheeledMobileRobot(WheeledMobileRobotInterface):
    def __init__(self, wheels):
        self._wheels = wheels


class MecanumWheel(WheelInterface):
    def __init__(self, gamma, radius):
        self._gamma = gamma
        self._radius = radius


class NonHolonomicWheel(WheelInterface):
    def __init__(self, radius):
        self._radius = radius
