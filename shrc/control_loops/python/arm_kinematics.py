import sympy
from sympy.physics import units
from sympy.physics import vector
import sympy.core.cache
import sympy.simplify.sqrtdenest
from sympy.simplify.sqrtdenest import sqrt_depth

import util

class ArmKinematics(object):
  """Represents the physical characteristics of the arm.

  This includes centers of mass, locations, etc for each segment and the motor
  constant for each one.

  This class also provides methods for calculating various relationships between
  the components (moment of inertia given the position of later segments etc).

  Terminology Notes:
    base: The part which anchors the arm.
    segment: A single rigid section which attaches with a motorized connection
             at one ends to the base, other segments, and/or the claw.
             The base end is closer to the base and the claw end is farther
             away from it. The rotation at the base end is considered part of
             the segment, but the rotation (if any) at the other end is
             considered part of the next segment.
    joint angles: A joint is at 0 when it is "straight".
    coordinates: Positive Z is up, positive Y is forwards with everything at 0,
                 and positive X is to the right.

  Attributes:
    segments: A sequence of Segments, in order from base to claw.
  """

  def __init__(self, segments):
    self.__segments = tuple(segments)

  @property
  def segments(self):
    return self.__segments

  def end_position(self, angles):
    """Calculates the position at the end of the Nth segment.

    Args:
      angles: A sequence of angles to use for each joint. The size of this
              sequence determines how many joints the position is calculated
              through.

    Returns a Position representing the end of the len(angles)th segment.
    """
    r = Position(vector.Vector(0), ARM_FRAME)
    for segment in zip(self.segments, angles):
      r = r.append(segment[0].position(segment[1]))
    return r

ARM_FRAME = vector.ReferenceFrame('N')

class Position(object):
  """Represents a position of a segment in the arm.

  A position is represented as a vector.Vector and a vector.ReferenceFrame.
  The reference frames goes at the end of the vector, and the actual "line"
  (ie tube) of the segment points down its Y axis.

  Attributes:
    vector: The vector.Vector in ARM_FRAME.
    direction: A unit vector.Vector in ARM_FRAME which represents the angle the
               end of the vector is pointing.
    angle: The amount clockwise (looking down direction) the end of the vector
           is turned.
  """

  """direction does not have to be normalized."""
  def __init__(self, vector, direction, angle):
    self.__vector = vector
    self.__direction = direction.normalize()
    self.__angle = angle

  @property
  def vector(self):
    return self.__vector
  @property
  def direction(self):
    return self.__direction
  @property
  def angle(self):
    return self.__angle

  def __repr__(self):
    return 'arm_kinematics.Position(%s, %s, %s)' % (self.vector.simplify(),
                                                    self.direction.simplify(),
                                                    sympy.simplify(self.angle))

  def __vector_fast_compare(self, a, b):
    if a == b:
      return True
    a_matrix = a.to_matrix(ARM_FRAME)
    b_matrix = b.to_matrix(ARM_FRAME)
    if a_matrix == b_matrix:
      return True
    found_difference = False
    for ea, eb in zip(a_matrix.tolist(), b_matrix.tolist()):
      ea = ea[0]
      eb = eb[0]
      if sympy.radsimp(ea) != sympy.radsimp(eb):
        found_difference = True
        break
    if not found_difference:
      return True
    return sympy.Eq(a_matrix, b_matrix) == 0

  @sympy.cache.cacheit
  def __eq__(self, other):
    if isinstance(other, Position):
      if sympy.Eq(other.angle, self.angle):
        if self.__vector_fast_compare(other.vector, self.vector):
          if self.__vector_fast_compare(other.direction, self.direction):
            return True
    return False

  @sympy.cache.cacheit
  def append(self, other):
    """Appends other to self in space."""
    rotated_frame = ARM_FRAME.orientnew('Nr', 'Axis', [self.angle, self.direction])
    if self.direction == rotated_frame.y:
      my_frame = rotated_frame
    else:
      my_frame_rotate = self.direction.cross(rotated_frame.y)
      angle = sympy.acos(self.direction.dot(rotated_frame.y))
      my_frame = rotated_frame.orientnew('Na', 'Axis', [angle, my_frame_rotate])
    my_dcm = my_frame.dcm(ARM_FRAME)
    rotated_vector = util.sequence_to_vector(
        ARM_FRAME,
        my_dcm * other.vector.to_matrix(ARM_FRAME))
    new_direction = util.sequence_to_vector(
        ARM_FRAME,
        my_dcm * other.direction.to_matrix(ARM_FRAME))
    return Position(self.vector + rotated_vector, new_direction,
                    self.angle + other.angle)

class Segment(object):
  """Represents one segment of the arm.

  Attributes:
    mass: The total mass of this segment in kg.
    cm: The distance from the base pivot to the center of mass in m.
    length: The distance between the two pivots in m.
  """

  def __init__(self, mass, cm, length):
    self.__mass = mass * units.kg
    self.__cm = cm * units.m
    self.__length = length * units.m

  @property
  def mass(self):
    return self.__mass
  @property
  def cm(self):
    return self.__cm
  @property
  def length(self):
    return self.__length

  def position(self, theta):
    """Returns:
      A Position representing this segment's current configuration (relative
      to the origin).
    """
    raise NotImplementedError

class PivotSegment(Segment):
  """Represents a segment which pivots at its base end.
  """

  def position(self, theta):
    frame = ARM_FRAME.orientnew('Np', 'Axis', [theta, ARM_FRAME.x])
    return Position(frame.dcm(ARM_FRAME) * ARM_FRAME.y * self.length, frame)

class TwistSegment(Segment):
  """Represents a segment which twists at its base end.
  """

  def position(self, theta):
    return Position(ARM_FRAME.y * self.length,
                    ARM_FRAME.orientnew('Nt', 'Axis', [theta, ARM_FRAME.y]))
