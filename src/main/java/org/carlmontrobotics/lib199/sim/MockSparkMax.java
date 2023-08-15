package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MockSparkMax extends MockedMotorBase {

    private static final ConcurrentHashMap<Integer, MockSparkMax> controllers = new ConcurrentHashMap<>();

    public final MotorType type;
    private final MockedEncoder encoder;
    private final SparkMaxPIDController pidController;
    private final MockedSparkMaxPIDController pidControllerImpl;
    private SparkMaxAbsoluteEncoder absoluteEncoder = null;
    private MockedEncoder alternateEncoder = null;
    private SparkMaxAnalogSensor analogSensor = null;

    public MockSparkMax(int port, MotorType type) {
        super("SparkMax", port, false);
        this.type = type;

        if(type == MotorType.kBrushless) {
            encoder = new MockedEncoder(SimDevice.create(device.getName() + "_RelativeEncoder"), MockedEncoder.builtinEncoderCountsPerRev, false) {
                @Override
                public REVLibError setInverted(boolean inverted) {
                    System.err.println(
                            "(MockedEncoder) SparkMaxRelativeEncoder cannot be inverted separately from the motor in brushless mode!");
                    return REVLibError.kParamInvalid;
                }
            };
        } else {
            encoder = new MockedEncoder(SimDevice.create(device.getName() + "_RelativeEncoder"), MockedEncoder.builtinEncoderCountsPerRev, false);
        }

        pidControllerImpl = new MockedSparkMaxPIDController(this);
        pidController = Mocks.createMock(SparkMaxPIDController.class, pidControllerImpl, new REVLibErrorAnswer());
        pidController.setFeedbackDevice(encoder);

        controllers.put(port, this);
    }

    @Override
    public double getRequestedSpeed() {
        return pidControllerImpl.calculate(getCurrentDraw());
    }

    public static MockSparkMax getControllerWithId(int port) {
        return controllers.get(port);
    }

    public static CANSparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(portPWM, type), new DummySparkMaxAnswer());
    }

    @Override
    public void set(double speed) {
        speed *= voltageCompensationNominalVoltage / defaultNominalVoltage;
        speed = (isInverted ? -1.0 : 1.0) * speed;
        pidControllerImpl.setDutyCycle(speed);
    }

    public REVLibError follow(CANSparkMax leader) {
        return follow(leader, false);
    }

    public REVLibError follow(CANSparkMax leader, boolean invert) {
		pidControllerImpl.follow(leader, invert); // No need to lookup the spark max if we already have it
        return REVLibError.kOk;
	}

    public REVLibError follow(ExternalFollower leader, int deviceID) {
        return follow(leader, deviceID, false);
    }

    public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
        MotorController controller = null;
        // Because ExternalFollower does not implement equals, this could result in bugs if the user passes in a custom ExternalFollower object,
        // but I think that it's unlikely and users should use the builtin definitions anyway
        if(leader.equals(ExternalFollower.kFollowerSparkMax)) {
            controller = getControllerWithId(deviceID);
        } else if(leader.equals(ExternalFollower.kFollowerPhoenix)) {
            // controller = MockPhoenixController.getControllerWithId(deviceID);
        }
        if(controller == null) {
            System.err.println("Error: Attempted to follow unknown motor controller: " + leader + " " + deviceID);
            return REVLibError.kFollowConfigMismatch;
        }
        pidControllerImpl.follow(controller, invert);
        return REVLibError.kOk;
    }

    public boolean isFollower() {
        return pidControllerImpl.isFollower();
    }

    public int getDeviceId() {
        return port;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type type, int countsPerRev) {
        if(type != Type.kHallSensor) {
            System.err.println("Error: MockSparkMax only supports hall effect encoders");
            return null;
        }
        return getEncoder();
    }

    @Override
    public void setInverted(boolean inverted) {
        super.setInverted(inverted);

        // Set the encoder inversion directly to avoid the error message
        if(type == MotorType.kBrushless) encoder.inverted = inverted;
    }

    public REVLibError enableVoltageCompensation(double nominalVoltage) {
        super.doEnableVoltageCompensation(nominalVoltage);
		return REVLibError.kOk;
	}

	public REVLibError disableVoltageCompensation() {
        super.doDisableVoltageCompensation();
		return REVLibError.kOk;
    }

    public SparkMaxPIDController getPIDController() {
        return pidController;
    }

    public double getAppliedOutput() {
        // MockedMotorBase returns speed before rate limiting.
        // The current output is the speed after rate limiting.
        return (isInverted ? -1.0 : 1.0) * speed.get();
    }

    public double getBusVoltage() {
        return defaultNominalVoltage;
    }

    @Override
    public void close() {
        controllers.remove(port);
        super.close();
    }

    public REVLibError enableSoftLimit​(CANSparkMax.SoftLimitDirection direction, boolean enable) {
        System.err.println("Error: MockSparkMax does not support soft limits");
        return REVLibError.kNotImplemented;
    }

    public synchronized SparkMaxAbsoluteEncoder getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type encoderType) {
        System.err.println("WARNING: An absolute encoder was created for a simulated Spark Max. Currently, the only way to specify the CPR is to use the REVHardwareClient. A CPR of " + MockedEncoder.builtinEncoderCountsPerRev + " will be assumed.");
        if(absoluteEncoder == null) {
            MockedEncoder absoluteEncoderImpl = new MockedEncoder(SimDevice.create(device.getName() + "_AbsoluteEncoder"), MockedEncoder.builtinEncoderCountsPerRev, true);
            absoluteEncoder = Mocks.createMock(SparkMaxAbsoluteEncoder.class, absoluteEncoderImpl, new REVLibErrorAnswer());
        }
        return absoluteEncoder;
    }

    public RelativeEncoder getAlternateEncoder(int countsPerRev) {
        return getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, countsPerRev);
    }

    public synchronized RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
        if(alternateEncoder == null) {
            alternateEncoder = new MockedEncoder(SimDevice.create(device.getName() + "_AlternateEncoder"), countsPerRev, false);
        }
        return alternateEncoder;
    }

    public synchronized SparkMaxAnalogSensor getAnalog(SparkMaxAnalogSensor.Mode mode) {
        if(analogSensor == null) {
            MockedEncoder analogSensorImpl = new MockedEncoder(SimDevice.create(device.getName() + "_AnalogSensor"), MockedEncoder.analogSensorCPR, true);
            analogSensor = Mocks.createMock(SparkMaxAnalogSensor.class, analogSensorImpl, new REVLibErrorAnswer());
        }
        return analogSensor;
    }

    public double getClosedLoopRampRate() {
        return getRampRateClosedLoop();
    }

    public double getOpenLoopRampRate() {
        return getRampRateOpenLoop();
    }

    public REVLibError setClosedLoopRampRate(double secondsFromNeutralToFull) {
        setRampRateClosedLoop(secondsFromNeutralToFull);
        return REVLibError.kOk;
    }

    public REVLibError setOpenLoopRampRate(double secondsFromNeutralToFull) {
        setRampRateOpenLoop(secondsFromNeutralToFull);
        return REVLibError.kOk;
    }

    public REVLibError setIdleMode(IdleMode mode) {
        super.setBrakeModeEnabled(mode == IdleMode.kBrake);
        return REVLibError.kOk;
    }

}
