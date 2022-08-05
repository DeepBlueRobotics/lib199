package org.carlmontrobotics.lib199.sim;

import com.revrobotics.REVLibError;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class MockedSparkEncoder implements AutoCloseable {
    private SimDevice device;
    private SimDouble dpp;
    private SimDouble count;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;

    public MockedSparkEncoder(int id) {
        // Match motor on CAN 0 with channels [0, 1], CAN 1 to channels [2, 3], etc.
        // Probably not the best way to do it but it works
        device = SimDevice.create("CANEncoder_SparkMax", id);
        dpp = device.createDouble("distancePerPulse", Direction.kOutput, 1);
        count = device.createDouble("count", Direction.kInput, 0);
    }

    public double getPosition() {
        return dpp.get() * count.get();
    }

    public REVLibError setPositionConversionFactor(double positionConversionFactor) {
        // Assume positionConversionFactor = units/rev
        // distancePerPulse (actually distance per count) = units/rev * rev/count
        dpp.set(positionConversionFactor / countsPerRevolution);
        return REVLibError.kOk;
    }

    public REVLibError setPosition(double position) {
        double revolutions = position / getPositionConversionFactor();
        count.set((int) Math.floor(revolutions * countsPerRevolution));
        return REVLibError.kOk;
    }

    public double getPositionConversionFactor() {
        return dpp.get() * countsPerRevolution;
    }

    @Override
    public void close() {
        device.close();
    }
}