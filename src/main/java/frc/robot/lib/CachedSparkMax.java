package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class CachedSparkMax extends CANSparkMax {

    private RelativeEncoder encoder;
    private SparkMaxPIDController pidController;

    public CachedSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        this.encoder = null;
        this.pidController = null;
    }

    @Override
    public RelativeEncoder getEncoder() {
        return encoder == null ? (encoder = super.getEncoder()) : encoder;
    }

    @Override
    public SparkMaxPIDController getPIDController() {
        return pidController == null ? (pidController = super.getPIDController()) : pidController;
    }

}
