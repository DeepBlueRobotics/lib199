package frc.robot.lib.Paths;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

//Findd Me
public class RobotPath {

    private Trajectory trajectory;
    private DrivetrainInterface dt;

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
    }

    public Command getPathCommand() {
        RamseteCommand ram = new RamseteCommand(trajectory, 
                                                () -> dt.getOdometry().getPoseMeters(), 
                                                new RamseteController(), 
                                                dt.getKinematics(),
                                                dt::charDriveDirect,
                                                dt);
        return new InstantCommand(this::loadOdometry).andThen(ram, new InstantCommand(() -> dt.charDriveTank(0, 0), dt)); //TODO: Configure Ramsete Controller Values
    }

    public void loadOdometry() {
        dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
    }

    public static TrajectoryConfig createConfig(boolean isInverted, DrivetrainInterface dt) {
        TrajectoryConfig config = new TrajectoryConfig(dt.getConfig().kAutoMaxSpeed, 
                                                       dt.getConfig().kAutoMaxAccel);
        config.setKinematics(dt.getKinematics());


        double kVoltAVG = 0.25 * (dt.getConfig().kVolts[0] + dt.getConfig().kVolts[1] + dt.getConfig().kVolts[2] + dt.getConfig().kVolts[3]);
        double kVelsAVG = 0.25 * (dt.getConfig().kVels[0] + dt.getConfig().kVels[1] + dt.getConfig().kVels[2] + dt.getConfig().kVels[3]);
        double kAccelAVG = 0.25 * (dt.getConfig().kAccels[0] + dt.getConfig().kAccels[1] + dt.getConfig().kAccels[2] + dt.getConfig().kAccels[3]);
        DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kVoltAVG, kVelsAVG, kAccelAVG), dt.getKinematics(), dt.getConfig().kAutoMaxVolt);
        config.addConstraint(voltConstraint);
        
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
}