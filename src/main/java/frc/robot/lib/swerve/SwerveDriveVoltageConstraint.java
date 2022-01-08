package frc.robot.lib.swerve;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

/**
 * A class that enforces constraints on swerve drivetrain voltage expenditure based on the motor
 * dynamics and the drive kinematics. Ensures that the acceleration of any wheel of the robot while
 * following the trajectory is never higher than what can be achieved with the given maximum
 * voltage. The code for this class was adapted from WPIlib's DifferentialDriveVoltageConstraint class.
 */
public class SwerveDriveVoltageConstraint implements TrajectoryConstraint {
  private final SimpleMotorFeedforward feedforward;
  private final SwerveDriveKinematics kinematics;
  private final double maxVoltage, trackWidth;

  /**
   * Creates a new SwerveDriveVoltageConstraint.
   *
   * @param feedforward A feedforward component describing the behavior of the drive.
   * @param kinematics A kinematics component describing the drive geometry.
   * @param maxVoltage The maximum voltage available to the motors while following the path. Should
   *     be somewhat less than the nominal battery voltage (12V) to account for "voltage sag" due to
   *     current draw.
   * @param trackWidth The track width of the drivetrain (in meters).
   */
  public SwerveDriveVoltageConstraint(SimpleMotorFeedforward feedforward, SwerveDriveKinematics kinematics,
                                      double maxVoltage, double trackWidth) {
    this.feedforward = requireNonNullParam(feedforward, "feedforward", "SwerveDriveVoltageConstraint");
    this.kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveDriveVoltageConstraint");
    this.maxVoltage = maxVoltage;
    this.trackWidth = trackWidth;
  }

  @Override
  public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    return Double.POSITIVE_INFINITY;
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(velocityMetersPerSecond * poseMeters.getRotation().getSin(), 
                                                          -velocityMetersPerSecond * poseMeters.getRotation().getCos(),
                                                          velocityMetersPerSecond * curvatureRadPerMeter));
    double[] wheelSpeeds = new double[states.length];
    for (int i = 0; i < states.length; i++) wheelSpeeds[i] = states[i].speedMetersPerSecond;

    double maxWheelSpeed = -Double.MAX_VALUE;
    double minWheelSpeed = Double.MAX_VALUE;
    for (double x : wheelSpeeds) {
        if (Math.abs(x) > maxWheelSpeed) maxWheelSpeed = x;
        if (Math.abs(x) < minWheelSpeed) minWheelSpeed = x;
    }

    // Calculate maximum/minimum possible accelerations from motor dynamics
    // and max/min wheel speeds
    double maxWheelAcceleration = feedforward.maxAchievableAcceleration(maxVoltage, maxWheelSpeed);
    double minWheelAcceleration = feedforward.minAchievableAcceleration(maxVoltage, minWheelSpeed);

    // Robot chassis turning on radius = 1/|curvature|.  Outer wheel has radius
    // increased by half of the trackwidth T.  Inner wheel has radius decreased
    // by half of the trackwidth.  Achassis / radius = Aouter / (radius + T/2), so
    // Achassis = Aouter * radius / (radius + T/2) = Aouter / (1 + |curvature|T/2).
    // Inner wheel is similar.

    // sgn(speed) term added to correctly account for which wheel is on
    // outside of turn:
    // If moving forward, max acceleration constraint corresponds to wheel on outside of turn
    // If moving backward, max acceleration constraint corresponds to wheel on inside of turn

    // When velocity is zero, then wheel velocities are uniformly zero (robot cannot be
    // turning on its center) - we have to treat this as a special case, as it breaks
    // the signum function.  Both max and min acceleration are *reduced in magnitude*
    // in this case.

    double maxChassisAcceleration, minChassisAcceleration;

    if (velocityMetersPerSecond == 0) {
      maxChassisAcceleration =
          maxWheelAcceleration / (1 + trackWidth * Math.abs(curvatureRadPerMeter) / 2);
      minChassisAcceleration =
          minWheelAcceleration / (1 + trackWidth * Math.abs(curvatureRadPerMeter) / 2);
    } else {
      maxChassisAcceleration =
            maxWheelAcceleration / (1 + trackWidth * Math.abs(curvatureRadPerMeter) * Math.signum(velocityMetersPerSecond) / 2);
      minChassisAcceleration =
            minWheelAcceleration / (1 - trackWidth * Math.abs(curvatureRadPerMeter) * Math.signum(velocityMetersPerSecond) / 2);
    }

    // When turning about a point inside of the wheelbase (i.e. radius less than half
    // the trackwidth), the inner wheel's direction changes, but the magnitude remains
    // the same.  The formula above changes sign for the inner wheel when this happens.
    // We can accurately account for this by simply negating the inner wheel.

    if ((trackWidth / 2) > (1 / Math.abs(curvatureRadPerMeter))) {
      if (velocityMetersPerSecond > 0) minChassisAcceleration = -minChassisAcceleration;
      else if (velocityMetersPerSecond < 0) maxChassisAcceleration = -maxChassisAcceleration;
    }

    return new MinMax(minChassisAcceleration, maxChassisAcceleration);
  }
}
