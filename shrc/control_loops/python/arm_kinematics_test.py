#!/usr/bin/python

import unittest
import cProfile

from sympy.physics import vector
import sympy

import arm_kinematics

class TestPosition(unittest.TestCase):
  def test_append_to_zero(self):
    original = arm_kinematics.Position(vector.Vector(0),
                                       arm_kinematics.ARM_FRAME.y,
                                       0)
    other = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.z,
        0)
    self.assertEqual(original.append(other), other)

  def test_append_to_straight(self):
    original = arm_kinematics.Position(5 * arm_kinematics.ARM_FRAME.y,
                                       arm_kinematics.ARM_FRAME.y,
                                       0)
    other = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        -sympy.cos(sympy.Rational(14, 10)) * arm_kinematics.ARM_FRAME.y +
        -sympy.sin(sympy.Rational(4, 10)) * arm_kinematics.ARM_FRAME.x, 0)
    result = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 7 * arm_kinematics.ARM_FRAME.y,
        -sympy.cos(sympy.Rational(14, 10)) * arm_kinematics.ARM_FRAME.y +
        -sympy.sin(sympy.Rational(4, 10)) * arm_kinematics.ARM_FRAME.x, 0)
    self.assertEqual(original.append(other), result)

  def test_append_float_angle(self):
    original = arm_kinematics.Position(5 * arm_kinematics.ARM_FRAME.y,
                                       arm_kinematics.ARM_FRAME.y,
                                       0)
    other = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        9.71 * arm_kinematics.ARM_FRAME.x +
        2.54 * arm_kinematics.ARM_FRAME.y,
        0)
    result = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 7 * arm_kinematics.ARM_FRAME.y,
        9.71 * arm_kinematics.ARM_FRAME.x +
        2.54 * arm_kinematics.ARM_FRAME.y,
        0)

  def test_append_straight(self):
    original = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.x + arm_kinematics.ARM_FRAME.y,
        0)
    other = arm_kinematics.Position(2 * arm_kinematics.ARM_FRAME.y,
                                    arm_kinematics.ARM_FRAME.y,
                                    0)
    result = arm_kinematics.Position(
        (1 + sympy.sqrt(2)) * arm_kinematics.ARM_FRAME.x +
        (2 + sympy.sqrt(2)) * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.x + arm_kinematics.ARM_FRAME.y,
        0)
    self.assertEqual(original.append(other), result)

  def test_append_straight_with_angle(self):
    original = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.x + arm_kinematics.ARM_FRAME.y,
        -sympy.pi / 2)
    other = arm_kinematics.Position(2 * arm_kinematics.ARM_FRAME.y,
                                    arm_kinematics.ARM_FRAME.y,
                                    0)
    result = arm_kinematics.Position(
        (1 + sympy.sqrt(2)) * arm_kinematics.ARM_FRAME.x +
        (2 + sympy.sqrt(2)) * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.x + arm_kinematics.ARM_FRAME.y,
        -sympy.pi / 2)
    self.assertEqual(original.append(other), result)

  def test_append_straight_with_useful_angle(self):
    original = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x + 2 * arm_kinematics.ARM_FRAME.y,
        arm_kinematics.ARM_FRAME.x + arm_kinematics.ARM_FRAME.y,
        -sympy.pi / 2)
    other = arm_kinematics.Position(2 * arm_kinematics.ARM_FRAME.x,
                                    arm_kinematics.ARM_FRAME.y,
                                    0)
    result = arm_kinematics.Position(
        arm_kinematics.ARM_FRAME.x +
        2 * arm_kinematics.ARM_FRAME.y +
        2 * arm_kinematics.ARM_FRAME.z,
        arm_kinematics.ARM_FRAME.x + arm_kinematics.ARM_FRAME.y,
        -sympy.pi / 2)
    self.assertEqual(original.append(other), result)

class TestArmKinematics(unittest.TestCase):
  """Also tests Segment, TwistSegment, and PivotSegment."""

  def setUp(self):
    R = sympy.Rational
    self.arm = arm_kinematics.ArmKinematics(
        (arm_kinematics.TurntableSegment(R(1, 10),
                                         R(15, 1000) * arm_kinematics.ARM_FRAME.y,
                                         R(3, 100)),
         arm_kinematics.PivotSegment(R(3, 10),
                                     R(1, 10) * arm_kinematics.ARM_FRAME.y,
                                     R(3, 10)),
         arm_kinematics.TwistSegment(R(3, 10),
                                     R(3, 100) * arm_kinematics.ARM_FRAME.y,
                                     R(1, 2)),
         arm_kinematics.PivotSegment(R(15, 100),
                                     R(1, 10) * arm_kinematics.ARM_FRAME.y,
                                     R(2, 10))))

  def test_end_position_all_zero(self):
    R = sympy.Rational
    self.assertEqual(self.arm.end_position((0,)),
                     arm_kinematics.Position(
                         R(3, 100) * arm_kinematics.ARM_FRAME.y,
                         arm_kinematics.ARM_FRAME.y, 0))
    self.assertEqual(self.arm.end_position((0, 0)),
                     arm_kinematics.Position(
                         R(33, 100) * arm_kinematics.ARM_FRAME.y,
                         arm_kinematics.ARM_FRAME.y, 0))
    self.assertEqual(self.arm.end_position((0, 0, 0)),
                     arm_kinematics.Position(
                         R(83, 100) * arm_kinematics.ARM_FRAME.y,
                         arm_kinematics.ARM_FRAME.y, 0))
    self.assertEqual(self.arm.end_position((0, 0, 0, 0)),
                     arm_kinematics.Position(
                         R(103, 100) * arm_kinematics.ARM_FRAME.y,
                         arm_kinematics.ARM_FRAME.y, 0))

  def test_end_position(self):
    R = sympy.Rational
    root2 = sympy.sqrt(2)
    self.assertEqual(self.arm.end_position((sympy.pi / 2,)),
                     arm_kinematics.Position(
                         R(3, 100) * arm_kinematics.ARM_FRAME.x,
                         arm_kinematics.ARM_FRAME.x, 0))
    self.assertEqual(self.arm.end_position((sympy.pi / 2, sympy.pi / 2)),
                     arm_kinematics.Position(
                         (R(3, 100) * arm_kinematics.ARM_FRAME.x +
                          R(30, 100) * arm_kinematics.ARM_FRAME.z),
                         arm_kinematics.ARM_FRAME.z, 0))
    self.assertEqual(self.arm.end_position((sympy.pi / 2, sympy.pi / 2,
                                            -sympy.pi / 2)),
                     arm_kinematics.Position(
                         (R(3, 100) * arm_kinematics.ARM_FRAME.x +
                          R(80, 100) * arm_kinematics.ARM_FRAME.z),
                         arm_kinematics.ARM_FRAME.z, -sympy.pi / 2))
    self.assertEqual(
        self.arm.end_position((sympy.pi / 2, sympy.pi / 2,
                               -sympy.pi / 2, -sympy.pi / 4)),
        arm_kinematics.Position(
            ((R(3, 100) + R(1, 10) * root2) * arm_kinematics.ARM_FRAME.x +
             (R(80, 100) + R(1, 10) * root2) * arm_kinematics.ARM_FRAME.z),
            arm_kinematics.ARM_FRAME.z + arm_kinematics.ARM_FRAME.x,
            -sympy.pi / 2))

  def test_angular_mass_all_zero(self):
    R = sympy.Rational
    self.assertEqual(self.arm.angular_mass(()), R(15, 10000))
    self.assertEqual(self.arm.angular_mass((0,)), 0)
    self.assertEqual(self.arm.angular_mass((0, 0)),
                     R(15717, 100000))
    self.assertEqual(self.arm.angular_mass((0, 0, 0)),
                     R(1737075, 10000000))

  def test_angular_mass_with_twist(self):
    R = sympy.Rational
    self.assertEqual(self.arm.angular_mass((0,)), 0)
    self.assertEqual(self.arm.angular_mass((sympy.pi / 4,)), R(75, 100000))

if __name__ == '__main__':
  #cProfile.run('unittest.main()', sort='cum')
  #cProfile.run('unittest.main()', sort='time')
  unittest.main()
