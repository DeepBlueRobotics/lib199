package frc.robot.lib.swerve;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A modification of WPIlib's RamseteCommand class which allows path-following on Swerve Drivetrains
 * 
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a swerve drivetrain.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class SwerveRamseteCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final boolean usePID;
  private final Trajectory trajectory;
  private final Supplier<Pose2d> pose;
  private final RamseteController follower;
  private final SimpleMotorFeedforward feedforward;
  private final SwerveDriveKinematics kinematics;
  private final Translation2d centerOfRotation;
  private final Supplier<Double[]> speeds;
  private final PIDController[] pidControllers;
  private final Consumer<SwerveModuleState[]> output;
  private SwerveModuleState[] prevStates;
  private double prevTime;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p> Additional note: The command will not normalize the computed speeds nor convert to
   * field-centric SwerveModuleStates. These actions must be taken by your Consumer<SwerveModuleState[]>
   * object.
   * 
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param centerOfRotation The position of the center of rotation from the robot's perspective. Set this
   *    argument to Translation2d() if do not want to rotate about a specific point on your robot.
   * @param speeds A function that supplies the speeds for each swerve module.
   * @param pidControllers The PID controllers for each swerve module on the drivetrain
   * @param output A function that consumes the computed outputs (in volts) for each swerve module.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("PMD.ExcessiveParameterList")
  public SwerveRamseteCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      SwerveDriveKinematics kinematics,
      Translation2d centerOfRotation,
      Supplier<Double[]> speeds,
      PIDController[] pidControllers,
      Consumer<SwerveModuleState[]> output,
      Subsystem... requirements) {
    this.trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveRamseteCommand");
    this.pose = requireNonNullParam(pose, "pose", "SwerveRamseteCommand");
    this.follower = requireNonNullParam(controller, "controller", "SwerveRamseteCommand");
    this.feedforward = feedforward;
    this.kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveRamseteCommand");
    this.centerOfRotation = requireNonNullParam(centerOfRotation, "centerOfRotation", "SwerveRamseteCommand");
    this.speeds = requireNonNullParam(speeds, "speeds", "SwerveRamseteCommand");
    this.pidControllers = requireNonNullParam(pidControllers, "pidControllers", "SwerveRamseteCommand");
    this.output = requireNonNullParam(output, "output", "SwerveRamseteCommand");

    usePID = true;

    addRequirements(requirements);
  }

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
   * the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * <p> Note: The command will not normalize the computed speeds nor convert to
   * field-centric SwerveModuleStates. These actions must be taken by your Consumer<SwerveModuleState[]>
   * object.
   * 
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to provide this.
   * @param follower The RAMSETE follower used to follow the trajectory.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param output A function that consumes the computed SwerveModuleState array
   * @param requirements The subsystems to require.
   */
  public SwerveRamseteCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController follower,
      SwerveDriveKinematics kinematics,
      Consumer<SwerveModuleState[]> output,
      Subsystem... requirements) {
    this.trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveRamseteCommand");
    this.pose = requireNonNullParam(pose, "pose", "SwerveRamseteCommand");
    this.follower = requireNonNullParam(follower, "follower", "SwerveRamseteCommand");
    this.kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveRamseteCommand");
    this.output = requireNonNullParam(output, "output", "SwerveRamseteCommand");

    feedforward = null;
    speeds = null;
    pidControllers = null;
    centerOfRotation = new Translation2d();

    usePID = false;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    Trajectory.State initialState = trajectory.sample(0);

    // +x corresponds to forward and +y corresponds to strafe left.
    // Keep in mind that this is NOT a traditional unit circle and that sin and cos are swapped
    // Heading
    double heading = initialState.poseMeters.getRotation().getRadians();
    // Forward velocity
    double vx = initialState.velocityMetersPerSecond * Math.sin(heading);
    // Strafe velocity
    double vy = -initialState.velocityMetersPerSecond * Math.cos(heading);
    // Angular velocity (rad/m * m/s = rad/s)
    double angularVel = initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond;
    // Inverse kinemtics: robot velocities ---> swerve module speed and angle
    prevStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, angularVel), centerOfRotation);
    
    timer.reset();
    timer.start();
    if (usePID) {
      for (PIDController pid : pidControllers) pid.reset();
    }
    prevTime = -1;
  }

  @Override
  public void execute() {
    double curTime = timer.get();
    double dt = curTime - prevTime;

    // We have no history - do nothing.
    if (prevTime < 0) {
        output.accept(new SwerveModuleState[pidControllers.length]);
        prevTime = curTime;
        return;
    }

    // Use the ramsete controller to compute the desired wheel speeds based on where the robot
    // currently is (pose) and where the robot needs to be (Trajectory.State)
    ChassisSpeeds targetWheelSpeeds = follower.calculate(pose.get(), trajectory.sample(curTime));

    SwerveModuleState[] swerveStates;
    double speed, targetSpeed, feed;
    // Use PID and feed forwards to add adjustments to the desired speeds to create a more accurate
    // estimate of what voltages we should supply to the modules.
    if (usePID) {
        // Inverse kinemtics: robot velocities ---> swerve module speed and angle
        swerveStates = kinematics.toSwerveModuleStates(targetWheelSpeeds, centerOfRotation);
        for (int i = 0; i < swerveStates.length; i++) {
            targetSpeed = swerveStates[i].speedMetersPerSecond;
            // Drivetrain characterization for each module - assume each module has approximately the same kS, kV, and kA.
            // TODO: Add functionality to account for cases where this assumption is false (especially kS, which may substantially differ).
            feed = feedforward.calculate(targetSpeed, (targetSpeed - prevStates[i].speedMetersPerSecond) / dt);
            // Add PID correction
            speed = feed + pidControllers[i].calculate(speeds.get()[i], targetSpeed);
            swerveStates[i].speedMetersPerSecond = speed;
        }
    } else swerveStates = kinematics.toSwerveModuleStates(targetWheelSpeeds, centerOfRotation);
    
    output.accept(swerveStates);
    prevStates = swerveStates;
    prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}