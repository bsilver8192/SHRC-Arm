#!/usr/bin/python

import unittest
import cProfile

from sympy.physics import vector
import sympy

import arm_kinematics

class TestPosition(unittest.TestCase):
  def test_append_to_zero(self):
    original = arm_kinematics.Position(vector.Vector(0),
                                       arm_kinematics.ARM_FRAME)
    other = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME)
    self.assertEqual(original.append(other), other)

  def test_append_to_straight(self):
    original = arm_kinematics.Position(5 * arm_kinematics.ARM_FRAME.y,
                                       arm_kinematics.ARM_FRAME)
    other = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.orientnew('Nt', 'Axis',
                                           [sympy.pi * sympy.Rational(14, 10),
                                            arm_kinematics.ARM_FRAME.z]))
    result = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 7 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.orientnew('Nt', 'Axis',
                                           [sympy.pi * sympy.Rational(14, 10),
                                            arm_kinematics.ARM_FRAME.z]))
    self.assertEqual(original.append(other), result)

  def test_append_float_angle(self):
    original = arm_kinematics.Position(5 * arm_kinematics.ARM_FRAME.y,
                                       arm_kinematics.ARM_FRAME)
    other = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.orientnew('Nt', 'Axis',
                                           [9.71, arm_kinematics.ARM_FRAME.z]))
    result = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 7 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.orientnew('Nt', 'Axis',
                                           [9.71, arm_kinematics.ARM_FRAME.z]))

  def test_append_straight(self):
    original = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.orientnew('Nt', 'Axis',
                                           [sympy.pi / 4,
                                            arm_kinematics.ARM_FRAME.z]))
    other = arm_kinematics.Position(2 * arm_kinematics.ARM_FRAME.y,
                                    arm_kinematics.ARM_FRAME)
    result = arm_kinematics.Position(
        (1 + sympy.sqrt(2)) * arm_kinematics.ARM_FRAME.x +
        (2 + sympy.sqrt(2)) * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.orientnew('Nt', 'Axis',
                                           [sympy.pi / 4,
                                            arm_kinematics.ARM_FRAME.z]))
    self.assertEqual(original.append(other), result)

if __name__ == '__main__':
  #cProfile.run('unittest.main()', sort='cum')
  #cProfile.run('unittest.main()', sort='time')
  unittest.main()
