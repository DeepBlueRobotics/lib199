package frc.robot.lib.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TrajectoryHeadingSupplier extends HeadingSupplier {
    private Trajectory trajectory;

    /**
     * Constructs a TrajectoryHeadingSupplier object
     * 
     * @param trajectory Represents a time-parameterized trajectory. The trajectory
     *                   contains of various States that represent the pose,
     *                   curvature, time elapsed, velocity, and acceleration at that
     *                   point.
     */
    public TrajectoryHeadingSupplier(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public Rotation2d getSample(double currentTime) {
        return trajectory.sample(currentTime).poseMeters.getRotation();
    }
}

