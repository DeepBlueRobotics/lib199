package frc.robot.lib;

import java.util.HashMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.sim.EncoderSim;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class MockSparkMax extends CANSparkMax {
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
    private final int portPWM;
    private final Spark motorPWM;
    private CANEncoder encoder;
    private boolean isInverted;
    // Since we need to keep a record of all the motor's followers
    private static HashMap<Integer, Spark> followMap = new HashMap<Integer, Spark>();

    public MockSparkMax(int portPWM, MotorType type) {
        super(portPWM, type);
        this.portPWM = portPWM;
        motorPWM = new Spark(portPWM);
        encoder = Mocks.createMock(CANEncoder.class, new MockedSparkEncoder(this));
        isInverted = false;
    }

    public static CANSparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(portPWM, type));
    }

    @Override
    public void set(double speed) {
        speed = (isInverted ? -1.0 : 1.0) * speed;
        motorPWM.set(speed);
        if (followMap.containsKey(portPWM)) followMap.get(portPWM).set(speed); 
    }

    @Override
    public CANError follow(CANSparkMax leader) {
        if (!followMap.containsValue(motorPWM)) followMap.put(leader.getDeviceId(), motorPWM);
        return CANError.kOk;
    }

    @Override
    public double get() {
        return motorPWM.get();
    }

    @Override
    public int getDeviceId() {
        return portPWM;
    }

    @Override
    public CANEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }
}

class MockedSparkEncoder extends CANEncoder {
    private Encoder encoder;
    private EncoderSim encoderSim;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;
    // Used for linking EncoderSim with Encoder
    private static int encodersCreated = 0;

    public MockedSparkEncoder(CANSparkMax motor) {
        super(motor);
        int id = motor.getDeviceId();
        // Match motor on CAN 0 with channels [0, 1], CAN 1 to channels [2, 3], etc.
        // Probably not the best way to do it but it works
        encoder = new Encoder(2 * id, 2 * id + 1);
        encoderSim = new EncoderSim(encodersCreated);
        encodersCreated++;
    }

    @Override
    public double getPosition() {
        return encoder.getDistance();
    }

    @Override
    public CANError setPositionConversionFactor(double positionConversionFactor) {
        // Assume positionConversionFactor = units/rev
        // distancePerPulse (actually distance per count) = units/rev * rev/count
        encoder.setDistancePerPulse(positionConversionFactor / countsPerRevolution);
        return CANError.kOk;
    }

    @Override
    public CANError setPosition(double position) {
        double revolutions = position / getPositionConversionFactor();
        encoderSim.setCount((int) Math.floor(revolutions * countsPerRevolution));
        return CANError.kOk;
    }

    @Override
    public double getPositionConversionFactor() {
        return encoder.getDistancePerPulse() * countsPerRevolution;
    }
}