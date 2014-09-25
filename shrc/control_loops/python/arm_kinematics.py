import itertools

import sympy
from sympy.physics import vector
import sympy.core.cache
import sympy.simplify.sqrtdenest
from sympy.simplify.sqrtdenest import sqrt_depth

import util
import lru_cache

ARM_FRAME = vector.ReferenceFrame('N')

class Position(object):
  """Represents a position of a segment in the arm.

  A position is represented as a vector.Vector for the position, another (unit)
  vector.Vector for the direction the end is facing, and an angle the end is
  rotated about that direction.
  The reference frames goes at the end of the vector, and the actual "line"
  (ie tube) of the segment points down its Y axis.

  Attributes:
    vector: The vector.Vector in ARM_FRAME.
    direction: A unit vector.Vector in ARM_FRAME which represents the angle the
               end of the vector is pointing.
    angle: The amount counter-clockwise (looking down direction) the end of the
           vector is turned.
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

  @staticmethod
  @lru_cache.lru_cache()
  def __vector_fast_compare(a, b):
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

  def __eq__(self, other):
    if isinstance(other, Position):
      if sympy.Eq(other.angle, self.angle):
        if Position.__vector_fast_compare(other.vector, self.vector):
          if Position.__vector_fast_compare(other.direction, self.direction):
            return True
    return False

  def __ne__(self, other):
    return not self == other

  def __hash__(self):
    return (hash(self.vector) * 101 + hash(self.direction) * 31 +
            hash(self.angle) * 17)

  @lru_cache.lru_cache()
  def __get_dcm(self, frame):
    """Returns the Direct Cosine Matrix to translate from frame to
    self.direction."""
    rotated_frame = ARM_FRAME.orientnew('Nr', 'Axis', [self.angle, self.direction])
    if self.direction == rotated_frame.y:
      # The Y axis already == self.direction, so we'll end up trying to rotate
      # around a point instead of a vector, which makes no sense, so just don't.
      my_frame = rotated_frame
    else:
      # Create a new frame whose Y axis == self.direction.
      my_frame_rotate = self.direction.cross(rotated_frame.y)
      angle = -sympy.acos(self.direction.dot(rotated_frame.y))
      my_frame = rotated_frame.orientnew('Na', 'Axis', [angle, my_frame_rotate])
    return frame.dcm(my_frame)

  @lru_cache.lru_cache()
  def append_vector(self, vector):
    """Appends vector to self in space."""
    return self.vector + util.sequence_to_vector(
        ARM_FRAME,
        self.__get_dcm(ARM_FRAME) * vector.to_matrix(ARM_FRAME))

  @lru_cache.lru_cache()
  def append(self, other):
    """Appends other to self in space."""
    my_dcm = self.__get_dcm(ARM_FRAME)
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
    cm: The vector from the base pivot to the center of mass (with the Y axis
        of the coordinate system pointing towards the claw pivot and the X axis
        straight up when the angle is 0). This vector is stored in ARM_FRAME.
    length: The distance between the two pivots in m.
    rotation_axis: A unit vector in ARM_FRAME which the segment rotates around.
  """

  def __init__(self, mass, cm, length):
    self.__mass = mass
    self.__cm = cm
    self.__length = length

  @property
  def mass(self):
    return self.__mass
  @property
  def cm(self):
    return self.__cm
  @property
  def length(self):
    return self.__length

  def rotation_axis(self):
    """Returns:
      A unit vector on the axis of rotation.
    """
    raise NotImplementedError

  def position(self, theta):
    """Returns:
      A Position representing this segment's current configuration (relative
      to the origin).
    """
    delta, angle = self._direction(theta)
    return Position(delta * self.length, delta, angle)

  def cm_at(self, theta):
    """Returns:
      A vector representing the location of the center of mass when the joint is
      at theta.
    """
    delta, _ = self._direction(theta)

    if delta != ARM_FRAME.y:
      # Create a new RefernceFrame whose Y axis == delta.
      delta_frame_rotate = delta.cross(ARM_FRAME.y)
      angle = -sympy.acos(delta.dot(ARM_FRAME.y))
      delta_frame = ARM_FRAME.orientnew('Na', 'Axis', [angle, delta_frame_rotate])

      return util.sequence_to_vector(
          ARM_FRAME,
          ARM_FRAME.dcm(delta_frame) * self.cm.to_matrix(ARM_FRAME))
    else:
      return self.cm

  def _direction(self, theta):
    """Returns:
      A unit vector representing the direction this segment is facing and the
      angle this segment will impart on the next segment."""
    raise NotImplementedError

class PivotSegment(Segment):
  """Represents a segment which pivots at its base end.

  0 theta is sticking straight out of the previous segment and positive is up.
  """

  def rotation_axis(self):
    return ARM_FRAME.x

  def _direction(self, theta):
    return sympy.sin(theta) * ARM_FRAME.z + sympy.cos(theta) * ARM_FRAME.y, 0

class TwistSegment(Segment):
  """Represents a segment which twists at its base end.

  The axis of the twisting is colinear with the endpoints. 0 theta means the
  ends line up and positive spins the claw end counter-clockwise (looking
  towards it).
  """

  def rotation_axis(self):
    return ARM_FRAME.y

  def _direction(self, theta):
    return ARM_FRAME.y, theta

class TurntableSegment(Segment):
  """Represents a segment which turns at its base end.

  The axis of the twisting is perpendicular to the claw end and coincident with
  the base end. 0 theta is pointing straight along the Y axis and positive turns
  to towards the X axis.
  """

  def rotation_axis(self):
    return ARM_FRAME.z

  def _direction(self, theta):
    return sympy.sin(theta) * ARM_FRAME.x + sympy.cos(theta) * ARM_FRAME.y, 0

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
              through. These angles start with the segment at the base end and
              work towards the claw end.

    Returns a Position representing the end of the len(angles)th segment.
    """
    r = Position(vector.Vector(0), ARM_FRAME.y, 0)
    for segment in zip(self.segments, angles):
      r = r.append(segment[0].position(segment[1]))
    return r

  def angular_mass(self, angles):
    """Calculates the moment of inertia for the Nth segment.

    Args:
      angles: A sequence of angles to use for each joint. The size of this
              sequence determines which joint the moment of inertia is
              calculated for. These angles start from the segment all the way
              at the claw end and work towards the base.

    Returns a sympy expression representing the moment of inertia for the
    [len(self.segments)-len(angles)-1]th segment (in kg*m^2).
    """
    position = Position(vector.Vector(0), ARM_FRAME.y, 0)
    r = 0
    rotation_axis = self.segments[-(len(angles) + 1)].rotation_axis()
    for segment in zip(self.segments[-(len(angles) + 1):],
                       itertools.chain([0], reversed(angles))):
      cm_vector = position.append_vector(segment[0].cm_at(segment[1]))
      radius = cm_vector.cross(rotation_axis).magnitude()
      r += segment[0].mass * radius**2
      position = position.append(segment[0].position(segment[1]))
    return r
