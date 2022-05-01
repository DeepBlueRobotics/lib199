package frc.robot.lib.sim;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.lib.Lib199Subsystem;
import edu.wpi.first.hal.SimDouble;

public class MockedCANCoder {

    public static final double kCANCoderCPR = 4096;

    private int port;
    private SimDevice device;
    private SimDouble position; // Rotations - Continuous
    private CANCoderSimCollection sim;

    public MockedCANCoder(CANCoder canCoder) {
        port = canCoder.getDeviceID();
        device = SimDevice.create("CANCoder", port);
        position = device.createDouble("count", Direction.kInput, 0);
        sim = canCoder.getSimCollection();
        Lib199Subsystem.registerPeriodic(this::update);
    }

    public void update() {
        sim.setRawPosition((int) (position.get() * kCANCoderCPR));
    }

}