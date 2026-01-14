package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;

import org.carlmontrobotics.lib199.Lib199Subsystem;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAnalogSensorSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * An extension of {@link MockedMotorBase} which implements spark-max-specific functionality
 */
public class MockSparkBase extends MockedMotorBase {

    private static final ConcurrentHashMap<Integer, MockSparkBase> controllers = new ConcurrentHashMap<>();

    public final MotorType type;
    private final MockedEncoder encoder;
    private final SparkBase motor;
    private final SparkSim spark;
    private final SparkClosedLoopController pidController;
    private final MockedSparkClosedLoopController pidControllerImpl;
    private SparkAbsoluteEncoder absoluteEncoder = null;
    private SparkAbsoluteEncoderSim absoluteEncoderImpl = null;
    private SparkMaxAlternateEncoder alternateEncoder = null;
    private SparkMaxAlternateEncoderSim alternateEncoderImpl = null;
    private SparkAnalogSensor analogSensor = null;
    private SparkAnalogSensorSim analogSensorImpl = null;
    private final String name;

    public enum NEOType { 
        NEO(DCMotor.getNEO(1)),
        NEO550(DCMotor.getNeo550(1)),
        VORTEX(DCMotor.getNeoVortex(1)),  
        UNKNOWN(DCMotor.getNEO(1));

        public DCMotor dcMotor;
        private NEOType(DCMotor dcmotordata){
            this.dcMotor=dcmotordata;
        }
    }

    /**
     * Initializes a new {@link SimDevice} with the given parameters and creates the necessary sim values, and
     * registers this class's {@link #run()} method to be called via {@link Lib199Subsystem#registerSimulationPeriodic(Runnable)}.
     *
     * @param port the port to associate this {@code MockSparkMax} with. Will be used to create the {@link SimDevice} and facilitate motor following.
     * @param type the type of the simulated motor. If this is set to {@link MotorType#kBrushless}, the builtin encoder simulation will be configured
     * to follow the inversion state of the motor and its {@code setInverted} method will be disabled.
     * @param name the name of the type of controller ("SparkMax" or "SparkFlex")
     * @param countsPerRev the number of counts per revolution of this controller's built-in encoder.
     * @param neoType the type of NEO motor
     */
    public MockSparkBase(int port, MotorType type, String name, int countsPerRev, NEOType neoType) {
        super(name, port);
        this.type = type;
        this.name = name;

        if (neoType == NEOType.VORTEX){ //only vortex uses sparkflex
            this.motor = new SparkFlex(port,type);
            this.spark = new SparkSim(
                this.motor,
                neoType.dcMotor
            );
        } else { //WARNING can't initialize a sparkbase without an actual spark...
            this.motor = new SparkMax(port,type);
            this.spark = new SparkSim(
                this.motor,
                neoType.dcMotor
            );
        }

        if(type == MotorType.kBrushless) {
            encoder = new MockedEncoder(SimDevice.create("CANEncoder:" + name, port), countsPerRev, false, false) {
                // @Override
                // public REVLibError setInverted(boolean inverted) {
                //     System.err.println(
                //             "(MockedEncoder) SparkRelativeEncoder cannot be inverted separately from the motor in brushless mode!");
                //     return REVLibError.kParamInvalid;
                // }
            };
        } else {
            encoder = new MockedEncoder(SimDevice.create("CANEncoder:" + name, port), countsPerRev, false, false);
        }

        pidControllerImpl = new MockedSparkClosedLoopController(this);
        pidController = Mocks.createMock(SparkClosedLoopController.class, pidControllerImpl, new REVLibErrorAnswer());
        // pidController.feedbackSensor(encoder);
        

        controllers.put(port, this);

        Lib199Subsystem.registerSimulationPeriodic(this);
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

    public REVLibError follow(SparkBase leader) {
        return follow(leader, false);
    }

    public REVLibError follow(SparkBase leader, boolean invert) {
		pidControllerImpl.follow(leader, invert); // No need to lookup the spark max if we already have it
        return REVLibError.kOk;
	}

    public REVLibError follow(SparkBase leader, int deviceID) {
        return follow(leader, deviceID, false);
    }

    public REVLibError follow(SparkBase leader, int deviceID, boolean invert) {
        MotorController controller = null;
        //ERROR: no way to check if leader is sending following frames or not
        controller = getControllerWithId(deviceID);
        if(controller == null) {
            System.err.println("Error: Attempted to follow unknown motor controller: " + leader + " " + deviceID);
            return REVLibError.kFollowConfigMismatch;
        }
        pidControllerImpl.follow(controller, invert);
        return REVLibError.kOk;
        /*
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
        */
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

    public SparkClosedLoopController getPIDController() {
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
        if (encoder != null) {
            encoder.close();
        }
        absoluteEncoderImpl=null;
        analogSensorImpl=null;
        alternateEncoder=null;
        super.close();
    }

    /**
     * Creates a simulated {@link SparkAbsoluteEncoder} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @return the simulated encoder
     */
    public synchronized SparkAbsoluteEncoder getAbsoluteEncoder() {
        if(absoluteEncoder == null) {
            if (motor instanceof SparkFlex){
                absoluteEncoderImpl = new SparkAbsoluteEncoderSim((SparkFlex)motor);
            } else {
                absoluteEncoderImpl = new SparkAbsoluteEncoderSim((SparkMax)motor);
            }
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
    public synchronized RelativeEncoder getAlternateEncoder(int countsPerRev) {
        // return getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, countsPerRev);
        if(alternateEncoder == null) {
            if (motor instanceof SparkFlex){
                System.err.println("Error: Attempted to get Alternate Encoder of a SparkFlex: " + motor.getDeviceId());
                return encoder;
            }
            alternateEncoderImpl = new SparkMaxAlternateEncoderSim((SparkMax)motor);
            alternateEncoder = Mocks.createMock(SparkMaxAlternateEncoder.class, absoluteEncoderImpl, new REVLibErrorAnswer());
        }
        return alternateEncoder;
    }

    /**
     * Creates a simulated {@link SparkAnalogSensor} linked to this simulated controller.
     * After this method has been called once, its output is cached for future invocations.
     * For this reason, the method is also {@code synchronized}.
     *
     * @return the simulated encoder
     */
    public synchronized SparkAnalogSensor getAnalog() {
        if(analogSensor == null) {
            if (motor instanceof SparkFlex){
                analogSensorImpl = new SparkAnalogSensorSim((SparkFlex)motor);
            } else {
                analogSensorImpl = new SparkAnalogSensorSim((SparkMax)motor);
            }
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

    public double getOutputCurrent() {
        return getCurrentDraw();
    }

    @Override
    public void disable() {
        // SparkBase sets the motor speed to zero rather than actually disabling the motor
        set(0);
    }

}
