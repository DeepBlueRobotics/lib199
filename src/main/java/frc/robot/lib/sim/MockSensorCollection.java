package frc.robot.lib.sim;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class MockSensorCollection {
    // The only sensors we will be using for the Phoenix motors are the Quadrature and Analog encoders.
    // Since Webots only has one type of position sensor, we have one sensor that must be able to act as either a quadrature or analog encoder.
    // Thus, we have a sim device whose type continually changes.
    private SimDevice encoder;
    private SimDouble quadCount, analogCount, analogMax;

    public MockSensorCollection(int deviceID) {
        encoder = SimDevice.create("PhoenixControllerSensor", deviceID);
        quadCount = encoder.createDouble("QuadEncoder Count", false, 0);
        analogCount = encoder.createDouble("AnalogEncoder Count", false, 0);
        analogMax = encoder.createDouble("AnalogEncoder Max", false, 1023);
    }

    public void setAnalogMax(int analogMax) {
        this.analogMax.set(analogMax);
    }

    public int getAnalogIn() {
        return getAnalogInRaw();
    }
    public int getAnalogInRaw() {
        return (int) analogCount.get();
    }

    public int getAnalogInVel() {
        return 0;
    }

    public ErrorCode setAnalogPosition(int newPosition, int timeoutMs) {
        return ErrorCode.OK;
    }

    public int getQuadratureVelocity() {
        return 0;
    }
    public int getQuadraturePosition() {
        return (int) quadCount.get();
    }
    public ErrorCode setQuadraturePosition(int newPosition, int timeoutMs) {
        return ErrorCode.OK;
    }

    /*
    #############
    Method Stubs
    #############
    */

    public int getPulseWidthPosition() { return 0; }
    public int getPulseWidthVelocity() { return 0; }
    public int getPulseWidthRiseToFallUs() { return 0; }
    public int getPulseWidthRiseToRiseUs() { return 0; }
    public ErrorCode syncQuadratureWithPulseWidth(int bookend0, int bookend1, boolean bCrossZeroOnInterval, int offset, int timeoutMs) { return ErrorCode.OK; }
    public ErrorCode syncQuadratureWithPulseWidth(int bookend0, int bookend1, boolean bCrossZeroOnInterval) { return ErrorCode.OK; }
    public ErrorCode setPulseWidthPosition(int newPosition, int timeoutMs) { return ErrorCode.OK; }

    public boolean getPinStateQuadA() { return false; }
    public boolean getPinStateQuadB() { return false; }
    public boolean getPinStateQuadIdx() { return false; }
    public boolean isFwdLimitSwitchClosed() { return false; }
    public boolean isRevLimitSwitchClosed() { return false; }
}
