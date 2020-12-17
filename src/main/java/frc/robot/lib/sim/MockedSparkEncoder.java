package frc.robot.lib.sim;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class MockedSparkEncoder extends CANEncoder {
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
        encoderSim = EncoderSim.createForIndex(encodersCreated);
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