package org.carlmontrobotics.lib199;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SparkVelocityPIDController implements Sendable {

    @SuppressWarnings("unused")
    private final SparkMax spark;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    private final String name;
    private double targetSpeed, tolerance;
    private double currentP, currentI, currentD, kS, kV;

    public SparkVelocityPIDController(String name, SparkMax spark, double defaultP, double defaultI, double defaultD, double kS, double kV, double targetSpeed, double tolerance) {
        this.spark = spark;
        this.pidController = spark.getClosedLoopController();
        this.encoder = spark.getEncoder();
        this.name = name;
        this.targetSpeed = targetSpeed;
        this.tolerance = tolerance;
        
        spark.configure(new SparkMaxConfig().apply(
            new ClosedLoopConfig().pid(
                this.currentP = defaultP,
                this.currentI = defaultI,
                this.currentD = defaultD
            )),
            SparkBase.ResetMode.kNoResetSafeParameters,//we only want to change pid params
            SparkBase.PersistMode.kNoPersistParameters);
        this.kS = kS;
        this.kV = kV;

        pidController.setReference(targetSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, calculateFF(targetSpeed));

        SendableRegistry.addLW(this, "SparkVelocityPIDController", spark.getDeviceId());
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
        this.targetSpeed = targetSpeed;
        pidController.setReference(targetSpeed, ControlType.kVelocity, 0, calculateFF(targetSpeed));
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculateFF(double velocity) {
        return kS * Math.signum(velocity) + kV * velocity;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSmartDashboardType("SparkVelocityPIDController");
        builder.addDoubleProperty("P", () -> currentP, p -> {
            pidController.setP(p);
            currentP = p;
        });
        builder.addDoubleProperty("I", () -> currentI, i -> {
            pidController.setI(i);
            currentI = i;
        });
        builder.addDoubleProperty("D", () -> currentD, d -> {
            pidController.setD(d);
            currentD = d;
        });
        builder.addDoubleProperty("Target Speed", () -> targetSpeed, newSpeed -> {
            if(newSpeed == targetSpeed) return;
            pidController.setReference(newSpeed, ControlType.kVelocity, 0, calculateFF(newSpeed));
            targetSpeed = newSpeed;
        });
        builder.addDoubleProperty("Tolerance", () -> tolerance, newTolerance -> tolerance = newTolerance);
        builder.addDoubleProperty("Current Speed", encoder::getVelocity, null);
    }

}
