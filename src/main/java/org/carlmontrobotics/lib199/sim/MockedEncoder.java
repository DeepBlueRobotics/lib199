package org.carlmontrobotics.lib199.sim;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.hal.SimDouble;

/**
 * Represents a base encoder class which can connect to a DeepBlueSim SimDeviceEncoderMediator.
 *
 * Currently this is only used for spark max simulation, pending #62. The REV classes just implement
 * methods defined in {@link AbsoluteEncoder}, {@link AnalogInput}, and {@link RelativeEncoder}, so
 * this class implements these interfaces to allow the compiler to check that all necessary methods
 * are implemented.
 *
 * This class can be used as a mock implementation when needed, but if possible, it should be used directly to reduce overhead.
 */
public class MockedEncoder implements AbsoluteEncoder, AnalogInput, AutoCloseable, RelativeEncoder {

    public static final double ANALOG_SENSOR_MAX_VOLTAGE = 3.3;

    public final SimDevice device;
    protected final SimDouble position;
    protected final SimDouble velocity;
    protected final SimDouble voltage;
    protected final SimBoolean init;
    protected final int countsPerRev;
    protected final boolean absolute;
    protected double positionConversionFactor = 1.0;
    protected double velocityConversionFactor = 1.0;
    protected double positionOffset = 0.0;
    protected boolean inverted = false;

    /**
     * @param device The device to retrieve position and velocity data from
     * @param countsPerRev The value that this.getCountsPerRevolution() should return
     * @param analog Whether the encoder is an analog sensor
     * @param absolute Whether the encoder is an absolute encoder.
     * This flag caps the position to one rotation via. {@link MathUtil#inputModulus(double, double, double)},
     * disables {@link #setPosition(double)}, and enables {@link #setZeroOffset(double)}.
     */
    public MockedEncoder(SimDevice device, int countsPerRev, boolean analog, boolean absolute) {
        this.device = device;
        position = device.createDouble("position", Direction.kInput, 0); // Rotations
        velocity = device.createDouble("velocity", Direction.kInput, 0); // Rotations per *second*
        if (analog) {
            voltage = device.createDouble("voltage", Direction.kInput, 0);
        } else {
            voltage = null;
        }
        this.countsPerRev = countsPerRev;
        this.absolute = absolute;
        init = device.createBoolean("init", Direction.kOutput, true);
    }

    @Override
    public REVLibError setPosition(double newPosition) {
        if (absolute) {
            System.err.println("(MockedEncoder) setPosition cannot be called on an absolute encoder");
            return REVLibError.kParamAccessMode;
        }
        positionOffset = newPosition - getRawPosition();
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

    /**
     * @return The current position of the encoder, not accounting for the position offset ({@link #setPosition(double)} and {@link #setZeroOffset(double)})
     */
    public double getRawPosition() {
        double rotationsOrVolts = voltage != null ? voltage.get() : position.get();
        return rotationsOrVolts * (inverted ? -1 : 1) * positionConversionFactor;
    }

    @Override
    public double getPosition() {
        if (absolute) {
            return MathUtil.inputModulus(getRawPosition() + positionOffset, 0, positionConversionFactor);
        } else {
            return getRawPosition() + positionOffset;
        }
    }

    @Override
    public double getVelocity() {
        return velocity.get() * 60 * (inverted ? -1 : 1) * velocityConversionFactor;
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
        init.set(false);
        device.close();
    }

    @Override
    public double getVoltage() {
        return voltage.get();
    }

}
