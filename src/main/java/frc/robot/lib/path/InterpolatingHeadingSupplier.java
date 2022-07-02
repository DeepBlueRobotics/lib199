package frc.robot.lib.path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class InterpolatingHeadingSupplier extends HeadingSupplier {

    private final Rotation2d initialRot;
    private final Rotation2d finalRot;
    private final double duration;

    public InterpolatingHeadingSupplier(Pose2d initialPose, Pose2d finalPose, double duration) {
        this(initialPose.getRotation(), finalPose.getRotation(), duration);
    }

    public InterpolatingHeadingSupplier(Rotation2d initialRot, Rotation2d finalRot, double duration) {
        this.initialRot = initialRot;
        this.finalRot = Rotation2d.fromDegrees(calculateFinalAngle(initialRot.getDegrees(), finalRot.getDegrees()));
        this.duration = duration;
    }

    /**
     * Calculates the final rotation so that the robot can turn the minimum distance
     *
     * @param initialAngleDeg The initial angle in degrees
     * @param finalAngleDeg The final angle in degrees
     *
     * @return The new final angle in degrees
     */
    public static double calculateFinalAngle(double initialAngleDeg, double finalAngleDeg) {
        while(initialAngleDeg - finalAngleDeg > 360) finalAngleDeg += 360;
        while(initialAngleDeg - finalAngleDeg < 0) finalAngleDeg -= 360;
        if(initialAngleDeg - finalAngleDeg > 180) finalAngleDeg += 360;
        return finalAngleDeg;
    }

    @Override
    protected Rotation2d getSample(double currentTime) {
        return initialRot.interpolate(finalRot, MathUtil.clamp(currentTime / duration, 0, 1));
    }

    public Rotation2d getInitialRot() {
        return initialRot;
    }

    public Rotation2d getFinalRot() {
        return finalRot;
    }

    public double getDuration() {
        return duration;
    }

}
