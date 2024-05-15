package org.carlmontrobotics.lib199;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

@Deprecated
public class CachedSparkMax extends CANSparkMax {

    private RelativeEncoder encoder;
    private SparkPIDController pidController;

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
    public SparkPIDController getPIDController() {
        return pidController == null ? (pidController = super.getPIDController()) : pidController;
    }

}
