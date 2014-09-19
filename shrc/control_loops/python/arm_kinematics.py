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
    frame: A vector.ReferenceFrame oriented to ARM_FRAME which represents the
            angle at the end of vector.
  """
    
  def __init__(self, vector, frame):
    self.__vector = vector
    self.__frame = frame

  @property
  def vector(self):
    return self.__vector
  @property
  def frame(self):
    return self.__frame

  def __repr__(self):
    return 'arm_kinematics.Position(%s, %s)' % (self.vector, self.frame)

  @sympy.cache.cacheit
  def __coordinate_eq(self, my_coordinate, other_coordinate):
    def check(expr):
      """Checks if expr is 0.

      Checking the kinds of expressions that this function deals with tends to
      be really really slow. This function tries some fast ways before getting
      to the really slow ones."""
      r = sympy.combsimp(sympy.radsimp(sympy.sqrtdenest(expr)))
      if r == 0:
        return True
      while sqrt_depth(r) > 1:
        denested = sympy.sqrtdenest(sympy.sqrt(r))
        if not denested.atoms(sympy.I):
          r = denested
        else:
          denested = sympy.sqrtdenest(sympy.sqrt(-r))
          if not denested.atoms(sympy.I):
            r = denested
          else:
            break
        if r == 0:
          return True
      # Hopefully we never get there. This takes on the order of 0.5s to run...
      return sympy.Eq(0, expr)
    other = other_coordinate.to_matrix(ARM_FRAME)
    mine = my_coordinate.to_matrix(ARM_FRAME)
    difference = other - mine
    return (check(difference[0]) and
            check(difference[1]) and
            check(difference[2]))
  @sympy.cache.cacheit
  def __eq__(self, other):
    if isinstance(other, Position):
      if other.vector == self.vector:
        if self.__coordinate_eq(self.frame.x, other.frame.x):
          if self.__coordinate_eq(self.frame.y, other.frame.y):
            if self.__coordinate_eq(self.frame.z, other.frame.z):
              return True
    return False

  @sympy.cache.cacheit
  def append(self, other):
    """Appends other to self in space."""
    new_vector = self.frame.dcm(ARM_FRAME) * other.vector.to_matrix(ARM_FRAME)

    new_frame = util.dcm_to_quaternion(
        self.frame.dcm(ARM_FRAME) * other.frame.dcm(ARM_FRAME))
    '''
    self_quat = util.dcm_to_quaternion(self.frame.dcm(ARM_FRAME))
    other_quat = util.dcm_to_quaternion(other.frame.dcm(ARM_FRAME))
    w1, x1, y1, z1 = self_quat
    w2, x2, y2, z2 = other_quat
    new_frame = (w1*w2 - x1*x2 - y1*y2 - z1*z2,
                 w1*x2 + x1*w2 + y1*z2 - z1*y2,
                 w1*y2 + y1*w2 + z1*x2 - x1*z2,
                 w1*z2 + z1*w2 + x1*y2 - y1*x2,
                 )
    '''

    return Position((ARM_FRAME.x * new_vector[0] +
                     ARM_FRAME.y * new_vector[1] +
                     ARM_FRAME.z * new_vector[2]) +
                    self.vector,
                    ARM_FRAME.orientnew('Na', 'Quaternion', new_frame))

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
