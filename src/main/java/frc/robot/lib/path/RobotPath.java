package frc.robot.lib.path;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

//Findd Me
public class RobotPath {

    private List<Pose2d> poses;
    private Trajectory trajectory;
    private TrajectoryConfig config;
    private DrivetrainInterface dt;
    private HeadingSupplier hs;
    private double maxAccelMps2;
    private double maxSpeedMps;

    /**
     * Constructs a RobotPath Object
     * @param pathName Name of the path
     * @param dt Drivetrain object
     * @param isInverted Whether the path is inverted
     * @param initPos Initial position
     * @throws IOException If the input wasn't valid
     */
    public RobotPath(String pathName, DrivetrainInterface dt, boolean isInverted, Translation2d initPos) throws IOException {
        this(getPointsFromFile(pathName, dt, isInverted, initPos), isInverted, dt);
    }

    /**
     * Constructs a RobotPath Object
     * @param poses List of points in the .path file
     * @param isInverted Whether the path is inverted
     * @param dt Drivetrain object
     */
    public RobotPath(List<Pose2d> poses, boolean isInverted, DrivetrainInterface dt) {
        this.poses = poses;
        this.dt = dt;
        this.maxAccelMps2 = dt.getMaxAccelMps2();
        this.maxSpeedMps = dt.getMaxSpeedMps();
    }

    /**
     * Gets a path command for the given path
     * @param faceInPathDirection Only for swerve drive, unless otherwise stated. Determines whether robot stays facing path
     * @param stopAtEnd whether the robot should stop at the end
     * @return PathCommand
     */
    public Command getPathCommand(boolean faceInPathDirection, boolean stopAtEnd) {
        if(trajectory == null) {
            generateTrajectory();
        }
        hs.reset();
        // We want the robot to stay facing the same direction (in this case), so save the current heading
        Rotation2d heading = Rotation2d.fromDegrees(dt.getHeadingDeg());
        Supplier<Rotation2d> desiredHeading = (!faceInPathDirection) ? () -> heading : () -> hs.sample();
        Command command = new InstantCommand(this::loadOdometry)
                .andThen(dt.createAutoCommand(trajectory, desiredHeading));
        if (stopAtEnd) {
            command = command.andThen(new InstantCommand(dt::stop, dt));
        }
        return command;
    }

    /**
     * Loads odometry
     */
    public void loadOdometry() {
        if(trajectory == null) {
            generateTrajectory();
        }
        dt.setOdometry(Rotation2d.fromDegrees(dt.getHeadingDeg()), trajectory.getInitialPose());
    }

    /**
     * Generates trajectory using List of poses and TrajectoryConfig objects
     */
    private void generateTrajectory() {
        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        hs = new HeadingSupplier(trajectory);
    }

    /**
     * Configures trajectory
     * @param configFunc TrajectoryConfig object
     */
    public TrajectoryConfig getTrajectoryConfig() {
        return config;
    }

    /**
     * Creates a TrajectoryConfig object
     * @param isInverted Whether path is inverted
     * @param dt Drivetrain object
     * @return TrajectoryConfig object
     */
    public TrajectoryConfig createConfig(boolean isInverted, DrivetrainInterface dt) {
        config = new TrajectoryConfig(this.getMaxSpeedMps(), this.getMaxAccelMps2());
        dt.configureAutoPath(this);

        if (isInverted) { config.setReversed(true); }

        return config;
    }

    /**
     * Get points of a path from name of a .path file
     * @param pathName Path name
     * @param dt Drivetrain object
     * @param isInverted Whether the path is inverted
     * @param initPos Initial position
     * @return List of points in path
     * @throws IOException
     */
    public static List<Pose2d> getPointsFromFile(String pathName, DrivetrainInterface dt, boolean isInverted, Translation2d initPos) throws IOException {
        return getPointsFromFile(getPathFile(pathName), dt, isInverted, initPos);
    }

    /**
     * Get points of a path from a .path file
     * @param file Filename
     * @param dt Drivetrain object
     * @param isInverted Whether the path is inverted
     * @param initPos Initial position
     * @return List of points in path
     * @throws IOException
     */
    public static List<Pose2d> getPointsFromFile(File file, DrivetrainInterface dt, boolean isInverted, Translation2d initPos) throws IOException {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        try {
            CSVParser csvParser = CSVFormat.DEFAULT.parse(new FileReader(file));
            double x, y, tanx, tany;
            Rotation2d rot;
            List<CSVRecord> records = csvParser.getRecords();

            for (int i = 1; i < records.size(); i++) {
                CSVRecord record = records.get(i);
                x = Double.parseDouble(record.get(0)) + initPos.getX();
                y = Double.parseDouble(record.get(1)) + initPos.getY();
                tanx = Double.parseDouble(record.get(2));
                tany = Double.parseDouble(record.get(3));
                rot = new Rotation2d(tanx, tany);
                if (isInverted) { rot = rot.rotateBy(new Rotation2d(Math.PI)); }
                poses.add(new Pose2d(x, y, rot));
            }
            csvParser.close();
        } catch (FileNotFoundException e) {
            System.out.println("File named: " + file.getAbsolutePath() + " not found.");
            e.printStackTrace();
        }

        return poses;
    }

    public double getMaxAccelMps2() {
        return this.maxAccelMps2;
    }

    public double getMaxSpeedMps() {
        return this.maxSpeedMps;
    }

    public void setMaxAccelMps2(double maxAccelMps2) {
        this.maxAccelMps2 = maxAccelMps2;
    }

    public void setMaxSpeedMps(double maxSpeedMps) {
        this.maxSpeedMps = maxSpeedMps;
    }

    /**
     * Gets .path file given filename
     * @param pathName name of file
     * @return .path file
     */
    public static File getPathFile(String pathName) {
        return Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".path")).toFile();
    }

    private static class HeadingSupplier {
        private Trajectory trajectory;
        private Timer timer;
        private boolean timerStarted;

        /**
         * Constructs a HeadingSupplier object
         * @param trajectory Represents a time-parameterized trajectory. The trajectory contains of various States that
         *                   represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
         */
        public HeadingSupplier(Trajectory trajectory) {
            this.trajectory = trajectory;
            timer = new Timer();
            timerStarted = false;
        }

        /**
         * Gets the trajectory rotation at current point in time
         * @return current trajectory rotation at current point in time
         */
        public Rotation2d sample() {
            if (!timerStarted) {
                timerStarted = true;
                timer.start();
            }
            return trajectory.sample(timer.get()).poseMeters.getRotation();
        }

        /**
         * Reset the timer
         */
        public void reset() {
            timerStarted = false;
            timer.reset();
        }
    }
}