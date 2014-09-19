from __future__ import print_function

import sympy
import sympy.core.cache

@sympy.cache.cacheit
def dcm_to_quaternion(dcm):
  """Converts a Direction Cosine Matrix to a Quaternion.

  Args:
    dcm: A 3x3 sympy.matrices.Matrix representing the DCM.

  Returns:
    A 4-tuple representing the 4 elements of the quaternion in order.
  """
  q0, q1, q2, q3 = my_symbols = sympy.symbols('q0 q1 q2 q3',
                                              imaginary=False,
                                              real=True)
  # This is basically [the formula for a DCM in terms of a quaternion] - dcm.
  # We can't just make a matrix out of it and subtract because sympy doesn't
  # currently support solving matrix equations.
  # Matrix from <http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion>.
  equations = (
      (q0**2 + q1**2 - q2**2 - q3**2) - dcm[0, 0],
      (2 * (q1 * q2 + q0 * q3)) - dcm[0, 1],
      (2 * (q1 * q3 - q0 * q2)) - dcm[0, 2],
      (2 * (q1 * q2 - q0 * q3)) - dcm[1, 0],
      (q0**2 - q1**2 + q2**2 - q3**2) - dcm[1, 1],
      (2 * (q2 * q3 + q0 * q1)) - dcm[1, 2],
      (2 * (q1 * q3 + q0 * q2)) - dcm[2, 0],
      (2 * (q2 * q3 - q0 * q1)) - dcm[2, 1],
      (q0**2 - q1**2 - q2**2 + q3**2) - dcm[2, 2],
      )
  result = sympy.solve(equations, my_symbols,
                       expand=False, rational=True, simplify=False)
  if not result:
    raise ValueError('Can not find a quaternion representing %s.' % dcm)
  return result[0]

def sequence_to_vector(frame, sequence):
  """Makes a vector out of a sequence.

  Args:
    frame: The sympy.physics.vector.ReferenceFrame to create the vector in.
    sequence: A 3-sequence of (x, y, z).

  Returns a new sympy.physics.vector.Vector in frame."""
  return (sequence[0] * frame.x +
          sequence[1] * frame.y +
          sequence[2] * frame.z)
