package frc.robot.lib.sim;

import com.revrobotics.CANError;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class MockedSparkEncoder {
    private SimDevice device;
    private SimDouble dpp;
    private SimDouble count;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;

    public MockedSparkEncoder(MockSparkMax motor) {
        int id = motor.getDeviceId();
        // Match motor on CAN 0 with channels [0, 1], CAN 1 to channels [2, 3], etc.
        // Probably not the best way to do it but it works
        device = SimDevice.create("CANEncoder_SparkMax", id);
        dpp = device.createDouble("distancePerPulse", false, 1);
        count = device.createDouble("count", false, 0);
    }

    public double getPosition() {
        return dpp.get() * count.get();
    }

    public CANError setPositionConversionFactor(double positionConversionFactor) {
        // Assume positionConversionFactor = units/rev
        // distancePerPulse (actually distance per count) = units/rev * rev/count
        dpp.set(positionConversionFactor / countsPerRevolution);
        return CANError.kOk;
    }

    public CANError setPosition(double position) {
        double revolutions = position / getPositionConversionFactor();
        count.set((int) Math.floor(revolutions * countsPerRevolution));
        return CANError.kOk;
    }

    public double getPositionConversionFactor() {
        return dpp.get() * countsPerRevolution;
    }
}