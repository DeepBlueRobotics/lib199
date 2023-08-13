package org.carlmontrobotics.lib199.sim;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.hal.SimDouble;

public class MockedEncoder implements AbsoluteEncoder, AnalogInput, AutoCloseable, RelativeEncoder {

    public static final int builtinEncoderCountsPerRev = 42;
    public static final double analogSensorMaxVoltage = 3.3;
    public static final int analogSensorCPR = 8192;

    public final SimDevice device;
    protected final SimDouble position;
    protected final SimDouble velocity;
    protected final int countsPerRev;
    protected final boolean absolute;
    protected double positionConversionFactor = 1.0;
    protected double velocityConversionFactor = 1.0;
    protected double positionOffset = 0.0;
    protected boolean inverted = false;

    public MockedEncoder(SimDevice device, int countsPerRev, boolean absolute) {
        this.device = device;
        position = device.createDouble("Position", Direction.kInput, 0);
        velocity = device.createDouble("Velocity", Direction.kInput, 0);
        this.countsPerRev = countsPerRev;
        this.absolute = absolute;
    }

    @Override
    public REVLibError setPosition(double position) {
        if (absolute) {
            System.err.println("(MockedEncoder) setPosition cannot be called on an absolute encoder");
            return REVLibError.kParamAccessMode;
        }
        positionOffset = position - getRawPosition();
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        System.err.println("(MockedEncoder) setMeasurementPeriod not implemented");
        return REVLibError.kNotImplemented;
    }

    @Override
    public int getMeasurementPeriod() {
        System.err.println("(MockedEncoder) getMeasurementPeriod not implemented");
        return 0;
    }

    @Override
    public int getCountsPerRevolution() {
        return countsPerRev;
    }

    public double getRawPosition() {
        return position.get() * (inverted ? -1 : 1) * positionConversionFactor / countsPerRev;
    }

    @Override
    public double getPosition() {
        if (absolute) {
            return MathUtil.inputModulus(getRawPosition(), 0, positionConversionFactor) + positionOffset;
        } else {
            return getRawPosition() + positionOffset;
        }
    }

    @Override
    public double getVelocity() {
        return velocity.get() * (inverted ? -1 : 1) * velocityConversionFactor / countsPerRev;
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        positionConversionFactor = factor;
        return REVLibError.kOk;
    }

    @Override
    public double getPositionConversionFactor() {
        return positionConversionFactor;
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        velocityConversionFactor = factor;
        return REVLibError.kOk;
    }

    @Override
    public double getVelocityConversionFactor() {
        return velocityConversionFactor;
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        this.inverted = inverted;
        return REVLibError.kOk;
    }

    public void setInvertedFromMotor(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        System.err.println("(MockedEncoder) setAverageDepth not implemented");
        return REVLibError.kNotImplemented;
    }

    @Override
    public int getAverageDepth() {
        System.err.println("(MockedEncoder) getAverageDepth not implemented");
        return 0;
    }

    @Override
    public REVLibError setZeroOffset(double offset) {
        if (!absolute) {
            System.err.println("(MockedEncoder) setZeroOffset cannot be called on a relative encoder");
            return REVLibError.kParamAccessMode;
        }
        positionOffset = offset;
        return REVLibError.kOk;
    }

    @Override
    public double getZeroOffset() {
        return positionOffset;
    }

    @Override
    public void close() {
        device.close();
    }

    @Override
    public double getVoltage() {
        return MathUtil.inputModulus(position.get() / countsPerRev, 0, 1) * analogSensorMaxVoltage;
    }

}
