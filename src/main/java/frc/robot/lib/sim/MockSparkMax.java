package frc.robot.lib.sim;

import java.util.HashMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.lib.Mocks;

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
        encoder = Mocks.createMock(CANEncoder.class, new MockedSparkEncoder(this, "PWM_" + portPWM));
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