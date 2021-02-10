package frc.robot.lib.sim;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.lib.Mocks;

public class MockTalonSRX extends WPI_TalonSRX {
    public final int port;

    public MockTalonSRX(int port) {
        super(port);
        this.port = port;
    }

    public static WPI_TalonSRX createMockTalonSRX(int port) {
        return Mocks.createMock(WPI_TalonSRX.class, new MockTalonSRX(port));
    }
}