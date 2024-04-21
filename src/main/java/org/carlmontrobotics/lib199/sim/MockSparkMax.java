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

/**
 * An extension of {@link MockedMotorBase} which implements spark-max-specific functionality
 */
public class MockSparkMax extends MockedMotorBase {

    private static final ConcurrentHashMap<Integer, MockSparkMax> controllers = new ConcurrentHashMap<>();

    public final MotorType type;
    private final MockedEncoder encoder;
    private final SparkMaxPIDController pidController;
    private final MockedSparkMaxPIDController pidControllerImpl;
    private SparkMaxAbsoluteEncoder absoluteEncoder = null;
    private MockedEncoder alternateEncoder = null;
    private SparkMaxAnalogSensor analogSensor = null;

    /**
     * @param port the port to associate this {@code MockSparkMax} with. Will be used to create the {@link SimDevice} and facilitate motor following.
     * @param type the type of the simulated motor. If this is set to {@link MotorType#kBrushless}, the builtin encoder simulation will be configured
     * to follow the inversion state of the motor and its {@code setInverted} method will be disabled.
     */
    public MockSparkMax(int port, MotorType type) {
        super("SparkMax", port);
        this.type = type;

        if(type == MotorType.kBrushless) {
            encoder = new MockedEncoder(SimDevice.create(device.getName() + "_RelativeEncoder"), MockedEncoder.NEO_BUILTIN_ENCODER_CPR, false) {
                @Override
                public REVLibError setInverted(boolean inverted) {
                    System.err.println(
                            "(MockedEncoder) SparkMaxRelativeEncoder cannot be inverted separately from the motor in brushless mode!");
                    return REVLibError.kParamInvalid;
                }
            };
        } else {
            encoder = new MockedEncoder(SimDevice.create(device.getName() + "_RelativeEncoder"), MockedEncoder.NEO_BUILTIN_ENCODER_CPR, false);
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

    /**
     * @param port the port of the controller to search for
     * @return Queries the simulated motor controller with the given port
     */
    public static MockSparkMax getControllerWithId(int port) {
        return controllers.get(port);
    }

    /**
     * Creates a simulated {@link CANSparkMax} with an instance of this class acting as the underling implementation, and forwarding all unimplemented method calls to {@link DummySparkMaxAnswer}
     * @param port the port to associate this {@code MockSparkMax} with. Will be used to create the {@link SimDevice} and facilitate motor following.
     * @param type the type of the simulated motor. If this is set to {@link MotorType#kBrushless}, the builtin encoder simulation will be configured
     * to follow the inversion state of the motor and its {@code setInverted} method will be disabled.
     * @return the simulated {@link CANSparkMax}
     */
    public static CANSparkMax createMockSparkMax(int port, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(port, type), new DummySparkMaxAnswer());
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
        if(leader.equals(ExternalFollower.kFollowerDisabled)) {
            pidControllerImpl.stopFollowing();
        } else {
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
        }
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

    /**
     * Creates a simulated {@link SparkMaxAbsoluteEncoder} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param encoderType ignored
     * @return the simulated encoder
     */
    public synchronized SparkMaxAbsoluteEncoder getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type encoderType) {
        System.err.println("WARNING: An absolute encoder was created for a simulated Spark Max. Currently, the only way to specify the CPR is to use the REVHardwareClient. A CPR of " + MockedEncoder.NEO_BUILTIN_ENCODER_CPR + " will be assumed.");
        if(absoluteEncoder == null) {
            MockedEncoder absoluteEncoderImpl = new MockedEncoder(SimDevice.create(device.getName() + "_AbsoluteEncoder"), MockedEncoder.NEO_BUILTIN_ENCODER_CPR, true);
            absoluteEncoder = Mocks.createMock(SparkMaxAbsoluteEncoder.class, absoluteEncoderImpl, new REVLibErrorAnswer());
        }
        return absoluteEncoder;
    }

    /**
     * Creates a simulated alternate encoder linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * This means that only the first call to this method will set the CPR of the encoder.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param countsPerRev the CPR of the absolute encoder
     * @return the simulated encoder
     */
    public RelativeEncoder getAlternateEncoder(int countsPerRev) {
        return getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, countsPerRev);
    }

    /**
     * Creates a simulated {@link SparkMaxAbsoluteEncoder} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param encoderType ignored
     * @return the simulated encoder
     */
    public synchronized RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
        if(alternateEncoder == null) {
            alternateEncoder = new MockedEncoder(SimDevice.create(device.getName() + "_AlternateEncoder"), countsPerRev, false);
        }
        return alternateEncoder;
    }

    /**
     * Creates a simulated {@link SparkMaxAnalogSensor} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param mode setting this to {@link SparkMaxAnalogSensor.Mode#kAbsolute} makes the position relative to the position on startup.
     * We will assume that this value is always zero, so this parameter has no effect.
     * @return the simulated encoder
     */
    public synchronized SparkMaxAnalogSensor getAnalog(SparkMaxAnalogSensor.Mode mode) {
        if(analogSensor == null) {
            MockedEncoder analogSensorImpl = new MockedEncoder(SimDevice.create(device.getName() + "_AnalogSensor"), MockedEncoder.ANALOG_SENSOR_MAX_VOLTAGE, true);
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

    @Override
    public void disable() {
        // CANSparkMax sets the motor speed to zero rather than actually disabling the motor
        set(0);
    }

}
