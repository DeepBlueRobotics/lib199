package org.carlmontrobotics.lib199.path;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean isInverted;

    /**
     * Constructs a RobotPath Object
     * 
     * @param pathName   Name of the path
     * @param dt         Drivetrain object
     * @param isInverted Whether the path is inverted
     * @param initPos    Initial position
     * @throws IOException If an error occured loading the path
     */
    public RobotPath(String pathName, DrivetrainInterface dt, boolean isInverted, Translation2d initPos)
            throws IOException {
        this(getPointsFromFile(pathName, dt, isInverted, initPos), isInverted, dt);

    }

    /**
     * Constructs a RobotPath Object
     * 
     * @param poses      List of points in the .path file
     * @param isInverted Whether the path is inverted
     * @param dt         Drivetrain object
     */
    public RobotPath(List<Pose2d> poses, boolean isInverted, DrivetrainInterface dt) {
        this.poses = poses;
        this.dt = dt;
        this.isInverted = isInverted;
        this.maxAccelMps2 = dt.getMaxAccelMps2();
        this.maxSpeedMps = dt.getMaxSpeedMps();
    }

    public RobotPath(List<Pose2d> poses, Trajectory trajectory, TrajectoryConfig config, DrivetrainInterface dt,
            HeadingSupplier hs, double maxAccelMps2, double maxSpeedMps, boolean isInverted) {
        this.poses = poses;
        this.trajectory = trajectory;
        this.config = config;
        this.dt = dt;
        this.hs = hs;
        this.maxAccelMps2 = maxAccelMps2;
        this.maxSpeedMps = maxSpeedMps;
        this.isInverted = isInverted;
    }

    public Rotation2d getRotation2d(int index){
        return poses.get(index).getRotation();
    }

    /**
     * Gets a path command for the given path
     * 
     * @param faceInPathDirection Only for swerve drive, unless otherwise stated.
     *                            Determines whether robot stays facing path
     * @param stopAtEnd           whether the robot should stop at the end
     * @return PathCommand
     */
    public Command getPathCommand(boolean faceInPathDirection, boolean stopAtEnd) {
        if (trajectory == null) {
            generateTrajectory();
        }
        // We want the robot to stay facing the same direction (in this case), so save
        // the current heading (make sure to update at the start of the command)
        AtomicReference<Rotation2d> headingRef = new AtomicReference<>(dt.getPose().getRotation());
        Supplier<Rotation2d> desiredHeading = (!faceInPathDirection) ? () -> headingRef.get() : () -> hs.sample();
        Command command = dt.createAutoCommand(trajectory, desiredHeading);
        command = new InstantCommand(hs::reset).andThen(command, new InstantCommand(hs::stop));
        if (stopAtEnd) {
            command = command.andThen(new InstantCommand(dt::stop, dt));
        }
        if (!faceInPathDirection) {
            command = new InstantCommand(() -> headingRef.set(dt.getPose().getRotation())).andThen(command);
            SmartDashboard.putNumber("Desired Path Heading", headingRef.get().getDegrees());
        }
        return command;
    }

    public Command getSwervePathCommand(boolean stopBetweenWaypoints, boolean stopAtEnd) {
        if(config == null) createConfig();

        Command command = new InstantCommand();

        for(int i = 0; i < poses.size() - 1; i++) {
            Pose2d initialPose = poses.get(i);
            Pose2d finalPose = poses.get(i + 1);

            Translation2d initialTranslation = initialPose.getTranslation();
            Translation2d finalTranslation = finalPose.getTranslation();
            Rotation2d initialRotation = initialPose.getRotation();
            Rotation2d finalRotatation = finalPose.getRotation();

            Rotation2d driveAngle = new Rotation2d(Math.atan2(finalTranslation.getY() - initialTranslation.getY(), finalTranslation.getX() - initialTranslation.getX()));

            List<Pose2d> newPoses = Arrays.asList(new Pose2d(initialTranslation, driveAngle), new Pose2d(finalTranslation, driveAngle));

            Trajectory newTrajectory = trajectory.concatenate(TrajectoryGenerator.generateTrajectory(newPoses, config));

            InterpolatingHeadingSupplier headingSupplier = new InterpolatingHeadingSupplier(initialRotation, finalRotatation, newTrajectory.getTotalTimeSeconds());

            command.andThen(new RobotPath(newPoses, newTrajectory, config, dt, headingSupplier, maxAccelMps2, maxSpeedMps, isInverted).getPathCommand(true, i == poses.size() - 1 ? stopAtEnd : stopBetweenWaypoints));
        }

        return command;
    }

    /**
     * Tells the drivetrain to assume that the robot is at the starting position of
     * this path.
     */
    public void initializeDrivetrainPosition() {
        if (trajectory == null) {
            generateTrajectory();
        }
        dt.setPose(trajectory.getInitialPose());
    }

    /**
     * Generates trajectory using List of poses and TrajectoryConfig objects
     */
    private void generateTrajectory() {
        if (config == null) {
            createConfig();
        }
        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        hs = new TrajectoryHeadingSupplier(trajectory);
    }

    /**
     * Retrieves the TrajectoryConfig object for this path, creating it if it does
     * not already exist
     * 
     * @return the TrajectoryConfig object
     */
    public TrajectoryConfig getTrajectoryConfig() {
        if (config == null) {
            createConfig();
        }
        return config;
    }

    private void createConfig() {
        config = new TrajectoryConfig(this.getMaxSpeedMps(), this.getMaxAccelMps2());
        dt.configureAutoPath(this);
        if (isInverted) {
            config.setReversed(true);
        }
    }

    /**
     * Inverts the robot path
     * 
     * @return inverted robot path
     */
    public RobotPath reversed() {
        List<Pose2d> newPoses = new ArrayList<>(poses);
        Collections.reverse(newPoses);
        return new RobotPath(newPoses, true, dt);
    }

    /**
     * Get points of a path from name of a .path file
     * 
     * @param pathName   Path name
     * @param dt         Drivetrain object
     * @param isInverted Whether the path is inverted
     * @param initPos    Initial position
     * @return List of points in path
     * @throws IOException If an error occured loading the path
     */
    public static List<Pose2d> getPointsFromFile(String pathName, DrivetrainInterface dt, boolean isInverted,
            Translation2d initPos) throws IOException {
        return getPointsFromFile(getPathFile(pathName), dt, isInverted, initPos);
    }

    /**
     * Get points of a path from a .path file
     * 
     * @param file       Filename
     * @param dt         Drivetrain object
     * @param isInverted Whether the path is inverted
     * @param initPos    Initial position
     * @return List of points in path
     * @throws IOException If an error occured loading the path
     */
    public static List<Pose2d> getPointsFromFile(File file, DrivetrainInterface dt, boolean isInverted,
            Translation2d initPos) throws IOException {
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
                if (isInverted) {
                    rot = rot.rotateBy(new Rotation2d(Math.PI));
                }
                poses.add(new Pose2d(x, y, rot));
            }
            csvParser.close();
        } catch (FileNotFoundException e) {
            System.out.println("File named: " + file.getAbsolutePath() + " not found.");
            e.printStackTrace();
        }

        return poses;
    }

    /**
     * Gets max acceleration for path
     * 
     * @return Max acceleration mps2
     */
    public double getMaxAccelMps2() {
        return this.maxAccelMps2;
    }

    /**
     * Gets max speed for path
     * 
     * @return Max speed mps
     */
    public double getMaxSpeedMps() {
        return this.maxSpeedMps;
    }

    /**
     * Sets max acceleration for path
     * 
     * @param maxAccelMps2 New max acceleration mps2
     * @return The robot path
     */
    public RobotPath setMaxAccelMps2(double maxAccelMps2) {
        checkConfig("maxAccelMps2");
        this.maxAccelMps2 = maxAccelMps2;
        return this;
    }

    /**
     * Sets max speed for path
     * 
     * @param maxSpeedMps New max speed mps
     * @return The robot path
     */
    public RobotPath setMaxSpeedMps(double maxSpeedMps) {
        checkConfig("maxSpeedMps");
        this.maxSpeedMps = maxSpeedMps;
        return this;
    }

    private void checkConfig(String varName) {
        if (config != null) {
            System.out.println(
                    "Warning: Config has already been created. The changes to " + varName + " will not affect it");
        }
    }

    /**
     * Gets .path file given filename
     * 
     * @param pathName name of file
     * @return .path file
     */
    public static File getPathFile(String pathName) {
        return Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".path"))
                .toFile();
    }

}
