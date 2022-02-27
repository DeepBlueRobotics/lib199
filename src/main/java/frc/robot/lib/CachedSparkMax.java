package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class CachedSparkMax extends CANSparkMax {

    private RelativeEncoder encoder;
    private SparkMaxPIDController pidController;

    public CachedSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        this.encoder = super.getEncoder();
        this.pidController = super.getPIDController();
    }

    @Override
    public RelativeEncoder getEncoder() {
        return encoder;
    }

    @Override
    public SparkMaxPIDController getPIDController() {
        return pidController;
    }

}
