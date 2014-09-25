#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Arm(control_loop.ControlLoop):
  def __init__(self, name="Arm"):
    super(Arm, self).__init__(name)
