package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class for SwerveMath - Team 199's custom implementation of methods for calculating the 
 *                        complicated mathematics involved in swerve driving, such as 
 *                        forward and inverse kinematics.
 *
 */
public class SwerveMath {
    /**
     * Robot-centric inverse kinematics: calculate SwerveModuleStates using desired forward, strafe, and rotation.
     * Inverse kinematics is essential in driving swerve drivetrain robots.
     * 
     * @param forward   The desired forward speed (in m/s) for the robot.
     * @param strafe    The desired strafe speed (in m/s) for the robot.
     * @param rotation  The desired rate of rotation (in rad/s) for the robot.
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot. 
     * @return An array consisting of four SwerveModuleState objects for the forward-left, forward-right, 
     *         backward-left, and backward-right modules.
     */
    public static SwerveModuleState[] calculateSwerve(double forward, double strafe, double rotation, double length, double width) {
        // Calculate helper variables
        double r = Math.sqrt(length * length + width * width);
        double a = strafe - rotation * (length / r);
        double b = strafe + rotation * (length / r);
        double c = forward - rotation * (width / r);
        double d = forward + rotation * (width / r);

        // Calculate wheel speeds for each side. SwerveMath does not normalize here in order to make inverseSwerve's math easier.
        double[] wheelSpeeds = {Math.sqrt(b * b + d * d), Math.sqrt(b * b + c * c),
                                Math.sqrt(a * a + d * d), Math.sqrt(a * a + c * c)};

        // Calculate angles for each side
        double[] angles = {Math.atan2(d, b), Math.atan2(c, b), Math.atan2(d, a), Math.atan2(c, a)};

        // Create and return SwerveModuleStates
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModuleStates[i] = new SwerveModuleState(wheelSpeeds[i], new Rotation2d(angles[i]));
        }
        return swerveModuleStates;
    }

    /**
     * Field-centric inverse kinematics: calculate SwerveModuleStates using desired forward, strafe, and rotation.
     * Inverse kinematics is essential in driving swerve drivetrain robots.
     * 
     * @param forward   The desired forward speed (in m/s) for the robot.
     * @param strafe    The desired strafe speed (in m/s) for the robot.
     * @param rotation  The desired rate of rotation (in rad/s) for the robot.
     * @param heading   The current heading of the robot (in rad) as measured by the robot's gyro.
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot. 
     * @return An array consisting of four SwerveModuleState objects for the forward-left, forward-right, 
     *         backward-left, and backward-right modules.
     */
    public static SwerveModuleState[] calculateSwerve(double forward, double strafe, double rotation, double heading, double length, double width) {
        double newForward = Math.cos(heading) * forward + Math.sin(heading) * strafe;
        double newStrafe = -Math.sin(heading) * forward + Math.cos(heading) * strafe;
        return calculateSwerve(newForward, newStrafe, rotation, length, width);
    }

    /**
     * Robot-centric forward kinematics: calculate the required forward, strafe, and rotation values needed to create known SwerveModuleStates.
     * Forward kinematics is useful in calculating the position of the robot and is used in odometry calculations.
     * 
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot.
     * @param states    Four SwerveModuleState objects in the order of forward-left, forward-right, backward-left, and backward-right.
     * @return An array consisting of the the corresponding forward, strafe, and rotation values measured in m/s, m/s, and rad/s respectively.
     */
    public static double[] inverseSwerve(double length, double width, SwerveModuleState ...states) {
        return inverseSwerve(length, width, 0, states);
    }

    /**
     * Field-centric forward kinematics: calculate the required forward, strafe, and rotation values needed to create known SwerveModuleStates.
     * Forward kinematics is useful in calculating the position of the robot and is used in odometry calculations.
     * 
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot.
     * @param heading   The current heading (in rad) as measured by the robot's gyro.
     * @param states    Four SwerveModuleState objects in the order of forward-left, forward-right, backward-left, and backward-right.
     * @return An array consisting of the the corresponding forward, strafe, and rotation values measured in m/s, m/s, and rad/s respectively.
     */
    public static double[] inverseSwerve(double length, double width, double heading, SwerveModuleState ...states) {
        // Calculate helper variables
        double a = states[3].speedMetersPerSecond * Math.cos(states[3].angle.getRadians());
        double b = states[0].speedMetersPerSecond * Math.cos(states[0].angle.getRadians());
        double c = states[1].speedMetersPerSecond * Math.sin(states[1].angle.getRadians());
        double d = states[2].speedMetersPerSecond * Math.sin(states[2].angle.getRadians());
        double r = Math.sqrt(length * length + width * width);

        // Calculate forward, strafe, and rotation
        double forward = (c + d) / 2;
        double strafe = (a + b) / 2;
        double rotation = (b - strafe) * (r / length);

        // Rotate from field-centric to robot-centric
        double newForward = Math.cos(heading) * forward - Math.sin(heading) * strafe;
        double newStrafe = Math.sin(heading) * forward + Math.cos(heading) * strafe;

        return new double[]{newForward, newStrafe, rotation};
    }

    /**
     * Computes the setpoint values for speed and angle for a singular motor controller.
     * 
     * @param normalizedSpeed   The desired normalized speed, from -1.0 to 1.0.
     * @param angle             The desired angle, from -1.0 to 1.0.
     * @param encoderPosition   The position of the <i> quadrature </i> encoder on the turn motor controller.
     * @param gearRatio         The gear ratio of the turn motor controller.
     * @return An array of doubles containing the setpoint values in the order of speed then angle.
     */
    public static double[] computeSetpoints(double normalizedSpeed, double angle, double encoderPosition, double gearRatio) {
        double newAngle = convertAngle(angle, encoderPosition, gearRatio);
        double speed = normalizedSpeed;
		
		if (shouldReverse(newAngle, encoderPosition, gearRatio)) {
			if (newAngle < 0) newAngle += 0.5;
			else newAngle -= 0.5;
			speed *= -1.0;
        }
		
		return new double[]{speed, newAngle};
    }

    /**
     * Determines whether or not the robot should take the reverse direction to get to angle. 
     * e.g. if the robot was to turn 3&#960/2 radians clockwise, it would be better to turn &#960/2 radians counter-clockwsie.
     * Credit to Team 100 for their code.
     * 
     * @param angle             The desired angle between -0.5 and 0.5
     * @param encoderPosition   The position of the <i> quadrature </i> encoder on the turn motor controller.
     * @param gearRatio        The gear ratio of the turn motor controller.
     * @return A boolean representing whether the robot should reverse or not.
     */
    public static boolean shouldReverse(double angle, double encoderPosition, double gearRatio){
        double convertedEncoder = (encoderPosition / gearRatio) % 1;
        // Convert the angle from -0.5 to 0.5 to 0 to 1.0
        if (angle < 0) angle += 1;
        
        double longDifference = Math.abs(angle - convertedEncoder);
        double difference = Math.min(longDifference, 1.0 - longDifference);

        // If the difference is greater than 1/4, then return true (aka it is easier for it to turn around and go backwards than go forward)
        if (difference > 0.25) return true;
        else return false;
    }

    /**
     * Converts the angle so that the robot can rotate in continuous circles. Credit to Team 100 for their code.
     * 
     * @param angle             The desired angle from -1.0 to 1.0
     * @param encoderPosition   The position of the <i> quadrature </i> encoder on the turn motor controller.
     * @param gearRatio        The gear ratio of the turn motor controller.
     * @return The converted angle between -0.5 and 0.5.
     */
    public static double convertAngle(double angle, double encoderPosition, double gearRatio) {
        double encPos = encoderPosition / gearRatio;

        double temp = angle;
        temp += (int)encPos;

        encPos = encPos % 1;

        if ((angle - encPos) > 0.5) temp -= 1;
        if ((angle - encPos) < -0.5) temp += 1;

        return temp;
    }
}