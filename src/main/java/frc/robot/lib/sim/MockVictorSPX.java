package frc.robot.lib.sim;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.lib.Mocks;

public class MockVictorSPX extends WPI_VictorSPX {
    public MockVictorSPX(int portPWM) {
        super(portPWM);
    }

    public static WPI_VictorSPX createMockVictorSPX(int port) {
        return Mocks.createMock(WPI_VictorSPX.class, new MockVictorSPX(port));
    }
}
