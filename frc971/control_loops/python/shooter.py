#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class SprungShooter(control_loop.ControlLoop):
  def __init__(self, name="RawSprungShooter"):
    super(SprungShooter, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = .4982
    # Stall Current in Amps
    self.stall_current = 85
    # Free Speed in RPM
    self.free_speed = 19300.0
    # Free Current in Amps
    self.free_current = 1.2
    # Effective mass of the shooter in kg.
    # This rough estimate should about include the effect of the masses
    # of the gears. If this number is too low, the eigen values of self.A
    # will start to become extremely small.
    self.J = 200
    # Resistance of the motor, divided by the number of motors.
    self.R = 12.0 / self.stall_current / 2.0
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Spring constant for the springs, N/m
    self.Ks = 2800.0
    # Maximum extension distance (Distance from the 0 force point on the
    # spring to the latch position.)
    self.max_extension = 0.32385
    # Gear ratio multiplied by radius of final sprocket.
    self.G = 10.0 / 40.0 * 20.0 / 54.0 * 24.0 / 54.0 * 20.0 / 84.0 * 16.0 * (3.0 / 8.0) / (2.0 * numpy.pi) * 0.0254

    # Control loop time step
    self.dt = 0.01

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[0, 1],
         [-self.Ks / self.J,
          -self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[0],
         [self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1, 0]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([0.45, 0.45])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl,
                             self.rpl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


class Shooter(SprungShooter):
  def __init__(self, name="RawShooter"):
    super(Shooter, self).__init__(name)

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[0, 1],
         [0, -self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[0],
         [self.Kt / (self.J * self.G * self.R)]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([0.45, 0.45])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl,
                             self.rpl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


class SprungShooterDeltaU(SprungShooter):
  def __init__(self, name="SprungShooter"):
    super(SprungShooterDeltaU, self).__init__(name)
    A_unaugmented = self.A
    B_unaugmented = self.B

    self.A = numpy.matrix([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0]])
    self.A[0:2, 0:2] = A_unaugmented
    self.A[0:2, 2] = B_unaugmented

    self.B = numpy.matrix([[0.0],
                           [0.0],
                           [1.0]])

    self.C = numpy.matrix([[1.0, 0.0, 0.0]])
    self.D = numpy.matrix([[0.0]])

    self.PlaceControllerPoles([0.50, 0.35, 0.80])

    print "K"
    print self.K
    print "Placed controller poles are"
    print numpy.linalg.eig(self.A - self.B * self.K)[0]

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl, 0.90])
    print "Placed observer poles are"
    print numpy.linalg.eig(self.A - self.L * self.C)[0]

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


class ShooterDeltaU(Shooter):
  def __init__(self, name="Shooter"):
    super(ShooterDeltaU, self).__init__(name)
    A_unaugmented = self.A
    B_unaugmented = self.B

    self.A = numpy.matrix([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0]])
    self.A[0:2, 0:2] = A_unaugmented
    self.A[0:2, 2] = B_unaugmented

    self.B = numpy.matrix([[0.0],
                           [0.0],
                           [1.0]])

    self.C = numpy.matrix([[1.0, 0.0, 0.0]])
    self.D = numpy.matrix([[0.0]])

    self.PlaceControllerPoles([0.55, 0.45, 0.80])

    print "K"
    print self.K
    print "Placed controller poles are"
    print numpy.linalg.eig(self.A - self.B * self.K)[0]

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl, 0.90])
    print "Placed observer poles are"
    print numpy.linalg.eig(self.A - self.L * self.C)[0]

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


def ClipDeltaU(shooter, old_voltage, delta_u):
  old_u = old_voltage
  new_u = numpy.clip(old_u + delta_u, shooter.U_min, shooter.U_max)
  return new_u - old_u

def main(argv):
  # Simulate the response of the system to a goal.
  sprung_shooter = SprungShooterDeltaU()
  raw_sprung_shooter = SprungShooter()
  close_loop_x = []
  close_loop_u = []
  goal_position = -0.3
  R = numpy.matrix([[goal_position], [0.0], [-sprung_shooter.A[1, 0] / sprung_shooter.A[1, 2] * goal_position]])
  voltage = numpy.matrix([[0.0]])
  for _ in xrange(500):
    U = sprung_shooter.K * (R - sprung_shooter.X_hat)
    U = ClipDeltaU(sprung_shooter, voltage, U)
    sprung_shooter.Y = raw_sprung_shooter.Y + 0.01
    sprung_shooter.UpdateObserver(U)
    voltage += U;
    raw_sprung_shooter.Update(voltage)
    close_loop_x.append(raw_sprung_shooter.X[0, 0] * 10)
    close_loop_u.append(voltage[0, 0])

  pylab.plot(range(500), close_loop_x)
  pylab.plot(range(500), close_loop_u)
  pylab.show()

  shooter = ShooterDeltaU()
  raw_shooter = Shooter()
  close_loop_x = []
  close_loop_u = []
  goal_position = -0.3
  R = numpy.matrix([[goal_position], [0.0], [-shooter.A[1, 0] / shooter.A[1, 2] * goal_position]])
  voltage = numpy.matrix([[0.0]])
  for _ in xrange(500):
    U = shooter.K * (R - shooter.X_hat)
    U = ClipDeltaU(shooter, voltage, U)
    shooter.Y = raw_shooter.Y + 0.01
    shooter.UpdateObserver(U)
    voltage += U;
    raw_shooter.Update(voltage)
    close_loop_x.append(raw_shooter.X[0, 0] * 10)
    close_loop_u.append(voltage[0, 0])

  pylab.plot(range(500), close_loop_x)
  pylab.plot(range(500), close_loop_u)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 5:
    print "Expected .h file name and .cc file name for"
    print "both the plant and unaugmented plant"
  else:
    unaug_sprung_shooter = SprungShooter("RawSprungShooter")
    unaug_shooter = Shooter("RawShooter")
    unaug_loop_writer = control_loop.ControlLoopWriter("RawShooter",
                                                       [unaug_sprung_shooter,
                                                        unaug_shooter])
    if argv[3][-3:] == '.cc':
      unaug_loop_writer.Write(argv[4], argv[3])
    else:
      unaug_loop_writer.Write(argv[3], argv[4])

    sprung_shooter = SprungShooterDeltaU()
    shooter = ShooterDeltaU()
    loop_writer = control_loop.ControlLoopWriter("Shooter", [sprung_shooter,
                                                             shooter])

    loop_writer.AddConstant(control_loop.Constant("kMaxExtension", "%f",
                                                  sprung_shooter.max_extension))
    loop_writer.AddConstant(control_loop.Constant("kSpringConstant", "%f",
                                                  sprung_shooter.Ks))
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
