package frc.robot.lib.path;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
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

    private Trajectory trajectory;
    private DrivetrainInterface dt;
    private HeadingSupplier hs;

    public RobotPath(String pathName, DrivetrainInterface dt, boolean isInverted, Translation2d initPos) throws IOException {
        this(getPointsFromFile(pathName, dt, isInverted, initPos), isInverted, dt);
    }

    public RobotPath(List<Pose2d> poses, boolean isInverted, DrivetrainInterface dt) {
        this(poses, createConfig(isInverted, dt), dt);
    }

    public RobotPath(List<Pose2d> poses, TrajectoryConfig config, DrivetrainInterface dt) {
        this(TrajectoryGenerator.generateTrajectory(poses, config), dt);
    }

    public RobotPath(Trajectory trajectory, DrivetrainInterface dt) {
        this.trajectory = trajectory;
        this.dt = dt;
        this.hs = new HeadingSupplier(trajectory);
    }

    public Command getPathCommand(boolean faceInPathDirection, boolean stopAtEnd) {
        hs.reset();
        // We want the robot to stay facing the same direction (in this case), so save the current heading
        Rotation2d heading = Rotation2d.fromDegrees(dt.getHeading());
        Supplier<Rotation2d> desiredHeading = (!faceInPathDirection) ? () -> heading : () -> hs.sample();
        Command command = new InstantCommand(this::loadOdometry).andThen(dt.createRamseteCommand(trajectory, desiredHeading));
        if(stopAtEnd) {
            command = command.andThen(new InstantCommand(dt::stop, dt));
        }
        return command;
    }

    public void loadOdometry() {
        dt.setOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose());
    }

    public static TrajectoryConfig createConfig(boolean isInverted, DrivetrainInterface dt) {
        TrajectoryConfig config = new TrajectoryConfig(dt.getAutoMaxSpeed(),
                                                       dt.getAutoMaxAccel());
        dt.configureTrajectory(config);

        if (isInverted) { config.setReversed(true); }

        return config;
    }

    public static List<Pose2d> getPointsFromFile(String pathName, DrivetrainInterface dt, boolean isInverted, Translation2d initPos) throws IOException {
        return getPointsFromFile(getPathFile(pathName), dt, isInverted, initPos);
    }

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

    public static File getPathFile(String pathName) {
        return Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".path")).toFile();
    }

    private static class HeadingSupplier {
        private Trajectory trajectory;
        private Timer timer;
        private boolean timerStarted;

        public HeadingSupplier(Trajectory trajectory) {
            this.trajectory = trajectory;
            timer = new Timer();
            timerStarted = false;
        }

        public Rotation2d sample() {
            if (!timerStarted) {
                timerStarted = true;
                timer.start();
            }
            return trajectory.sample(timer.get()).poseMeters.getRotation();
        }

        public void reset() {
            timerStarted = false;
            timer.reset();
        }
    }
}