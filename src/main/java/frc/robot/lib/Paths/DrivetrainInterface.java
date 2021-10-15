package frc.robot.lib.Paths;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DrivetrainInterface extends Subsystem {
   
    public Config getConfig();

    public DifferentialDriveOdometry getOdometry();

    public DifferentialDriveKinematics getKinematics();

    public void setOdometry(DifferentialDriveOdometry odometry);

    public void charDriveDirect(double left, double right);

    public void charDriveTank(double left, double right);

    public double getHeading();

    public static class Config {
        public double[] kVolts;
        public double[] kVels;
        public double[] kAccels;
        public double kAutoMaxAccel;
        public double kAutoMaxSpeed;
        public double kAutoMaxVolt;

        public Config(double[] kVolts, double[] kVels, double[] kAccels, double kAutoMaxAccel, double kAutoMaxSpeed,
                double kAutoMaxVolt) {
            this.kVolts = kVolts;
            this.kVels = kVels;
            this.kAccels = kAccels;
            this.kAutoMaxAccel = kAutoMaxAccel;
            this.kAutoMaxSpeed = kAutoMaxSpeed;
            this.kAutoMaxVolt = kAutoMaxVolt;
        }

        
    }

}
