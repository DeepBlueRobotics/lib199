package org.carlmontrobotics.lib199;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

@Deprecated
public class CachedSparkMax extends SparkMax {

    private RelativeEncoder encoder;
    private SparkClosedLoopController pidController;

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
    public SparkClosedLoopController getClosedLoopController() {
        return pidController == null ? (pidController = super.getClosedLoopController()) : pidController;
    }

}
