package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkVelocityPIDController {

    private final CANSparkMax spark;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private final String name;
    private double targetSpeed, tolerance;
    private double currentP, currentI, currentD, kS, kV;

    public SparkVelocityPIDController(String name, CANSparkMax spark, double defaultP, double defaultI, double defaultD, double kS, double kV, double targetSpeed, double tolerance) {
        this.spark = spark;
        this.pidController = spark.getPIDController();
        this.encoder = spark.getEncoder();
        this.name = name;
        this.targetSpeed = targetSpeed;
        this.tolerance = tolerance;
        pidController.setP(this.currentP = defaultP);
        pidController.setI(this.currentI = defaultI);
        pidController.setD(this.currentD = defaultD);
        this.kS = kS;
        this.kV = kV;

        SmartDashboard.putNumber(name + ": P", currentP);
        SmartDashboard.putNumber(name + ": I", currentI);
        SmartDashboard.putNumber(name + ": D", currentD);
        SmartDashboard.putNumber(name + ": Target Speed", targetSpeed);
        SmartDashboard.putNumber(name + ": Tolerance", tolerance);

        pidController.setReference(targetSpeed, ControlType.kVelocity, 0, calculateFF(targetSpeed));
    }

    public void periodic() {
        double p = SmartDashboard.getNumber(name + ": P", currentP);
        double i = SmartDashboard.getNumber(name + ": I", currentI);
        double d = SmartDashboard.getNumber(name + ": D", currentD);

        if(p != currentP) {
            pidController.setP(p);
            currentP = p;
        }
        if(i != currentI) {
            pidController.setI(i);
            currentI = i;
        }
        if(d != currentD) {
            pidController.setD(d);
            currentD = d;
        }

        setTargetSpeed(SmartDashboard.getNumber(name + ": Target Speed", targetSpeed));
        setTolerance(SmartDashboard.getNumber(name + ": Tolerance", tolerance));
        SmartDashboard.putNumber(name + ": Current Speed", encoder.getVelocity());
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public boolean isAtTargetSpeed() {
        return encoder.getVelocity() > targetSpeed - tolerance;
    }

    public String getName() {
        return name;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed) {
        if(targetSpeed == this.targetSpeed) return;
        SmartDashboard.putNumber(name + ": Target Speed", targetSpeed);
        this.targetSpeed = targetSpeed;
        pidController.setReference(targetSpeed, ControlType.kVelocity, 0, calculateFF(targetSpeed));
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        SmartDashboard.putNumber(name + ": Tolerance", tolerance);
        this.tolerance = tolerance;
    }

    public double calculateFF(double velocity) {
        return kS * Math.signum(velocity) + kV * velocity;
    }

}
