package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;

import org.carlmontrobotics.lib199.Lib199Subsystem;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * An extension of {@link MockedMotorBase} which implements spark-max-specific functionality
 */
public class MockSparkBase extends MockedMotorBase {

    private static final ConcurrentHashMap<Integer, MockSparkBase> controllers = new ConcurrentHashMap<>();

    public final MotorType type;
    private final MockedEncoder encoder;
    private final SparkPIDController pidController;
    private final MockedSparkMaxPIDController pidControllerImpl;
    private SparkAbsoluteEncoder absoluteEncoder = null;
    private MockedEncoder alternateEncoder = null;
    private SparkAnalogSensor analogSensor = null;
    private final String name;

    /**
     * Initializes a new {@link SimDevice} with the given parameters and creates the necessary sim values, and
     * registers this class's {@link #run()} method to be called asynchronously via {@link Lib199Subsystem#registerAsyncSimulationPeriodic(Runnable)}.
     *
     * @param port the port to associate this {@code MockSparkMax} with. Will be used to create the {@link SimDevice} and facilitate motor following.
     * @param type the type of the simulated motor. If this is set to {@link MotorType#kBrushless}, the builtin encoder simulation will be configured
     * to follow the inversion state of the motor and its {@code setInverted} method will be disabled.
     * @param name the name of the type of controller ("CANSparkMax" or "CANSparkFlex")
     */
    public MockSparkBase(int port, MotorType type, String name, int countsPerRev) {
        super(name, port);
        this.type = type;
        this.name = name;

        if(type == MotorType.kBrushless) {
            encoder = new MockedEncoder(SimDevice.create("CANEncoder:%s[%d]".formatted(name, port)), countsPerRev, false, false) {
                @Override
                public REVLibError setInverted(boolean inverted) {
                    System.err.println(
                            "(MockedEncoder) SparkRelativeEncoder cannot be inverted separately from the motor in brushless mode!");
                    return REVLibError.kParamInvalid;
                }
            };
        } else {
            encoder = new MockedEncoder(SimDevice.create("CANEncoder:%s[%d]".formatted(name, port)), countsPerRev, false, false);
        }

        pidControllerImpl = new MockedSparkMaxPIDController(this);
        pidController = Mocks.createMock(SparkPIDController.class, pidControllerImpl, new REVLibErrorAnswer());
        pidController.setFeedbackDevice(encoder);

        controllers.put(port, this);

        Lib199Subsystem.registerAsyncSimulationPeriodic(this);
    }

    @Override
    public double getRequestedSpeed() {
        return pidControllerImpl.calculate(getCurrentDraw());
    }

    /**
     * @param port the port of the controller to search for
     * @return Queries the simulated motor controller with the given port
     */
    public static MockSparkBase getControllerWithId(int port) {
        return controllers.get(port);
    }

    @Override
    public void set(double speed) {
        speed *= voltageCompensationNominalVoltage / defaultNominalVoltage;
        pidControllerImpl.setDutyCycle(speed);
    }

    public REVLibError follow(CANSparkBase leader) {
        return follow(leader, false);
    }

    public REVLibError follow(CANSparkBase leader, boolean invert) {
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
            if(leader.equals(ExternalFollower.kFollowerSpark)) {
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

    public RelativeEncoder getEncoder(SparkRelativeEncoder.Type type, int countsPerRev) {
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

    public SparkPIDController getPIDController() {
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
     * Creates a simulated {@link SparkAbsoluteEncoder} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param encoderType ignored
     * @return the simulated encoder
     */
    public synchronized SparkAbsoluteEncoder getAbsoluteEncoder(SparkAbsoluteEncoder.Type encoderType) {
        if(absoluteEncoder == null) {
            MockedEncoder absoluteEncoderImpl = new MockedEncoder(SimDevice.create("CANDutyCycle:%s[%d]".formatted(name, port)), 0, false, true);
            absoluteEncoder = Mocks.createMock(SparkAbsoluteEncoder.class, absoluteEncoderImpl, new REVLibErrorAnswer());
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
     * Creates a simulated {@link SparkMaxAlternateEncoder} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param encoderType ignored
     * @return the simulated encoder
     */
    public synchronized RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
        if(alternateEncoder == null) {
            alternateEncoder = new MockedEncoder(SimDevice.create("CANEncoder:%s[%d]-alternate".formatted(name, port)), 0, false, false);
        }
        return alternateEncoder;
    }

    /**
     * Creates a simulated {@link SparkAnalogSensor} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @param mode setting this to {@link SparkAnalogSensor.Mode#kAbsolute} makes the position relative to the position on startup.
     * We will assume that this value is always zero, so this parameter has no effect.
     * @return the simulated encoder
     */
    public synchronized SparkAnalogSensor getAnalog(SparkAnalogSensor.Mode mode) {
        if(analogSensor == null) {
            MockedEncoder analogSensorImpl = new MockedEncoder(SimDevice.create("CANAIn:%s[%d]".formatted(name, port)), 0, true, true);
            analogSensor = Mocks.createMock(SparkAnalogSensor.class, analogSensorImpl, new REVLibErrorAnswer());
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
        // CANSparkBase sets the motor speed to zero rather than actually disabling the motor
        set(0);
    }

}
